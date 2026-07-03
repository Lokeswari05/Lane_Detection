
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity lane_detect is
    port (
        clk        : in  std_logic;
        reset_n    : in  std_logic;
        enable_in  : in  std_logic_vector(2 downto 0);
        vs_in      : in  std_logic;
        hs_in      : in  std_logic;
        de_in      : in  std_logic;
        r_in       : in  std_logic_vector(7 downto 0);
        g_in       : in  std_logic_vector(7 downto 0);
        b_in       : in  std_logic_vector(7 downto 0);
        vs_out     : out std_logic;
        hs_out     : out std_logic;
        de_out     : out std_logic;
        r_out      : out std_logic_vector(7 downto 0);
        g_out      : out std_logic_vector(7 downto 0);
        b_out      : out std_logic_vector(7 downto 0);
        clk_o      : out std_logic;
        led        : out std_logic_vector(2 downto 0)
    );
end lane_detect;

architecture rtl of lane_detect is

    signal reset : std_logic;

    -- Stage 1: gray
    signal s1_de,s1_hs,s1_vs : std_logic;
    signal s1_gray            : std_logic_vector(7 downto 0);

    -- Stage 2: roi
    signal s2_de,s2_hs,s2_vs : std_logic;
    signal s2_gray            : std_logic_vector(7 downto 0);
    signal s2_roi             : std_logic;
    signal s2_row             : integer range 0 to V_ACTIVE-1;

    -- Stage 3a: line buffer taps
    signal s3_de,s3_hs,s3_vs : std_logic;
    signal tp0,tp1,tp2,tp3,tp4,tp5,tp6 : std_logic_vector(7 downto 0);

    -- Stage 3b: edge
    signal s4_de,s4_hs,s4_vs : std_logic;
    signal s4_roi             : std_logic;
    signal s4_ebin            : std_logic;
    signal s4_emag            : std_logic_vector(7 downto 0);

    -- Stage 4: blob
    signal s5_de,s5_hs,s5_vs : std_logic;
    signal bv,bc_s            : std_logic;
    signal b_valid            : std_logic;
    signal b_count            : integer range 0 to MAX_BLOBS;
    signal b_row              : integer range 0 to V_ACTIVE-1;
    signal b_x,b_x1           : integer range 0 to H_ACTIVE-1;
    signal b_x2,b_x3,b_x4    : integer range 0 to H_ACTIVE-1;
    signal b_x5,b_x6,b_x7    : integer range 0 to H_ACTIVE-1;

    -- RANSAC
    signal ll_v,lr_v          : std_logic;
    signal ll_a,ll_b          : integer range -65536 to 65535;
    signal lr_a,lr_b          : integer range -65536 to 65535;

    -- Pipeline delay for RGB + control (total = 8 clocks)
    -- rgb2gray=1, roi=1, lbuf=1, sdct=1, blob=1 + 3 spare = 8
    constant PIPE_D : integer := 8;
    type rgb_pipe_t is array(0 to PIPE_D-1) of std_logic_vector(23 downto 0);
    signal rgb_pipe : rgb_pipe_t := (others=>(others=>'0'));

    type ctrl_pipe_t is array(0 to PIPE_D-1) of std_logic_vector(4 downto 0);
    signal ctrl_pipe : ctrl_pipe_t := (others=>(others=>'0'));

    -- ROI and row delay
    type row_pipe_t is array(0 to PIPE_D-1) of integer range 0 to V_ACTIVE-1;
    signal row_pipe : row_pipe_t := (others=>0);
    type roi_pipe_t is array(0 to PIPE_D-1) of std_logic;
    signal roi_pipe : roi_pipe_t := (others=>'0');

    -- Stage 6 overlay
    signal s6_de,s6_hs,s6_vs : std_logic;
    signal s6_r,s6_g,s6_b    : std_logic_vector(7 downto 0);

    -- Components
    component rgb2gray is
        port(clk,de_in,hs_in,vs_in:in std_logic;
             r_in,g_in,b_in:in std_logic_vector(7 downto 0);
             r_d,g_d,b_d:out std_logic_vector(7 downto 0);
             de_out,hs_out,vs_out:out std_logic;
             gray:out std_logic_vector(7 downto 0));
    end component;
    signal dummy_r,dummy_g,dummy_b : std_logic_vector(7 downto 0);

    component roi_mask is
        port(clk,reset:in std_logic;
             de_in,hs_in,vs_in:in std_logic;
             gray_in,r_in,g_in,b_in:in std_logic_vector(7 downto 0);
             de_out,hs_out,vs_out:out std_logic;
             gray_out,r_out,g_out,b_out:out std_logic_vector(7 downto 0);
             roi_active:out std_logic;
             row_num:out integer range 0 to V_ACTIVE-1);
    end component;
    signal s2_r,s2_g,s2_b : std_logic_vector(7 downto 0);

    component line_buffer_7tap is
        port(clk,reset:in std_logic;
             de_in,hs_in,vs_in:in std_logic;
             data_in:in std_logic_vector(7 downto 0);
             de_out,hs_out,vs_out:out std_logic;
             tap0,tap1,tap2,tap3,tap4,tap5,tap6:out std_logic_vector(7 downto 0));
    end component;

    component sdct_edge is
        port(clk,reset:in std_logic;
             de_in,hs_in,vs_in,roi_in:in std_logic;
             tap0,tap1,tap2,tap3,tap4,tap5,tap6:in std_logic_vector(7 downto 0);
             de_out,hs_out,vs_out,roi_out:out std_logic;
             edge_bin:out std_logic;
             edge_mag:out std_logic_vector(7 downto 0));
    end component;

    component blob_analysis is
        port(clk,reset:in std_logic;
             de_in,hs_in,vs_in,roi_in:in std_logic;
             edge_bin:in std_logic;
             row_in:in integer range 0 to V_ACTIVE-1;
             de_out,hs_out,vs_out:out std_logic;
             blob_valid:out std_logic;
             blob_count:out integer range 0 to MAX_BLOBS;
             blob_row:out integer range 0 to V_ACTIVE-1;
             blob_x,blob_x1,blob_x2,blob_x3:out integer range 0 to H_ACTIVE-1;
             blob_x4,blob_x5,blob_x6,blob_x7:out integer range 0 to H_ACTIVE-1);
    end component;

    component ransac_line is
        port(clk,reset,vs_in:in std_logic;
             blob_valid:in std_logic;
             blob_count:in integer range 0 to MAX_BLOBS;
             blob_row:in integer range 0 to V_ACTIVE-1;
             blob_x,blob_x1,blob_x2,blob_x3:in integer range 0 to H_ACTIVE-1;
             blob_x4,blob_x5,blob_x6,blob_x7:in integer range 0 to H_ACTIVE-1;
             lane_l_valid,lane_r_valid:out std_logic;
             lane_l_a,lane_l_b:out integer range -65536 to 65535;
             lane_r_a,lane_r_b:out integer range -65536 to 65535);
    end component;

    component lane_overlay is
        port(clk:in std_logic;
             de_in,hs_in,vs_in:in std_logic;
             r_in,g_in,b_in:in std_logic_vector(7 downto 0);
             roi_in:in std_logic;
             row_in:in integer range 0 to V_ACTIVE-1;
             lane_l_valid,lane_r_valid:in std_logic;
             lane_l_a,lane_l_b:in integer range -65536 to 65535;
             lane_r_a,lane_r_b:in integer range -65536 to 65535;
             de_out,hs_out,vs_out:out std_logic;
             r_out,g_out,b_out:out std_logic_vector(7 downto 0));
    end component;

begin
    reset  <= not reset_n;
    clk_o  <= clk;
    led(0) <= ll_v;
    led(1) <= lr_v;
    led(2) <= '0';

    u_gray: rgb2gray port map(
        clk=>clk, de_in=>de_in, hs_in=>hs_in, vs_in=>vs_in,
        r_in=>r_in, g_in=>g_in, b_in=>b_in,
        r_d=>dummy_r, g_d=>dummy_g, b_d=>dummy_b,
        de_out=>s1_de, hs_out=>s1_hs, vs_out=>s1_vs, gray=>s1_gray);

    u_roi: roi_mask port map(
        clk=>clk, reset=>reset,
        de_in=>s1_de, hs_in=>s1_hs, vs_in=>s1_vs,
        gray_in=>s1_gray,
        r_in=>(others=>'0'), g_in=>(others=>'0'), b_in=>(others=>'0'),
        de_out=>s2_de, hs_out=>s2_hs, vs_out=>s2_vs,
        gray_out=>s2_gray,
        r_out=>s2_r, g_out=>s2_g, b_out=>s2_b,
        roi_active=>s2_roi, row_num=>s2_row);

    u_lbuf: line_buffer_7tap port map(
        clk=>clk, reset=>reset,
        de_in=>s2_de, hs_in=>s2_hs, vs_in=>s2_vs,
        data_in=>s2_gray,
        de_out=>s3_de, hs_out=>s3_hs, vs_out=>s3_vs,
        tap0=>tp0,tap1=>tp1,tap2=>tp2,tap3=>tp3,
        tap4=>tp4,tap5=>tp5,tap6=>tp6);

    u_sdct: sdct_edge port map(
        clk=>clk, reset=>reset,
        de_in=>s3_de, hs_in=>s3_hs, vs_in=>s3_vs,
        roi_in=>s2_roi,
        tap0=>tp0,tap1=>tp1,tap2=>tp2,tap3=>tp3,
        tap4=>tp4,tap5=>tp5,tap6=>tp6,
        de_out=>s4_de, hs_out=>s4_hs, vs_out=>s4_vs,
        roi_out=>s4_roi,
        edge_bin=>s4_ebin, edge_mag=>s4_emag);

    u_blob: blob_analysis port map(
        clk=>clk, reset=>reset,
        de_in=>s4_de, hs_in=>s4_hs, vs_in=>s4_vs,
        roi_in=>s4_roi, edge_bin=>s4_ebin,
        row_in=>s2_row,
        de_out=>s5_de, hs_out=>s5_hs, vs_out=>s5_vs,
        blob_valid=>b_valid, blob_count=>b_count, blob_row=>b_row,
        blob_x=>b_x, blob_x1=>b_x1,
        blob_x2=>b_x2, blob_x3=>b_x3,
        blob_x4=>b_x4, blob_x5=>b_x5,
        blob_x6=>b_x6, blob_x7=>b_x7);

    u_ransac: ransac_line port map(
        clk=>clk, reset=>reset, vs_in=>vs_in,
        blob_valid=>b_valid, blob_count=>b_count, blob_row=>b_row,
        blob_x=>b_x, blob_x1=>b_x1,
        blob_x2=>b_x2, blob_x3=>b_x3,
        blob_x4=>b_x4, blob_x5=>b_x5,
        blob_x6=>b_x6, blob_x7=>b_x7,
        lane_l_valid=>ll_v, lane_r_valid=>lr_v,
        lane_l_a=>ll_a, lane_l_b=>ll_b,
        lane_r_a=>lr_a, lane_r_b=>lr_b);

    -- Combined RGB + control delay pipeline
    process(clk)
    begin
        if rising_edge(clk) then
            -- Pack RGB into one 24-bit shift register
            rgb_pipe(0) <= r_in & g_in & b_in;
            for i in 1 to PIPE_D-1 loop
                rgb_pipe(i) <= rgb_pipe(i-1);
            end loop;
            -- Control: vs,hs,de,roi,dummy
            ctrl_pipe(0) <= vs_in & hs_in & de_in & s2_roi & '0';
            for i in 1 to PIPE_D-1 loop
                ctrl_pipe(i) <= ctrl_pipe(i-1);
            end loop;
            -- Row delay
            row_pipe(0) <= s2_row;
            for i in 1 to PIPE_D-1 loop
                row_pipe(i) <= row_pipe(i-1);
            end loop;
        end if;
    end process;

    u_overlay: lane_overlay port map(
        clk=>clk,
        de_in=>ctrl_pipe(PIPE_D-1)(2),
        hs_in=>ctrl_pipe(PIPE_D-1)(3),
        vs_in=>ctrl_pipe(PIPE_D-1)(4),
        r_in=>rgb_pipe(PIPE_D-1)(23 downto 16),
        g_in=>rgb_pipe(PIPE_D-1)(15 downto 8),
        b_in=>rgb_pipe(PIPE_D-1)(7  downto 0),
        roi_in=>ctrl_pipe(PIPE_D-1)(1),
        row_in=>row_pipe(PIPE_D-1),
        lane_l_valid=>ll_v, lane_r_valid=>lr_v,
        lane_l_a=>ll_a, lane_l_b=>ll_b,
        lane_r_a=>lr_a, lane_r_b=>lr_b,
        de_out=>s6_de, hs_out=>s6_hs, vs_out=>s6_vs,
        r_out=>s6_r, g_out=>s6_g, b_out=>s6_b);

    -- Output
    process(clk)
    begin
        if rising_edge(clk) then
            vs_out <= s6_vs; hs_out <= s6_hs; de_out <= s6_de;
            case enable_in is
                when "000" =>
                    r_out<=s1_gray; g_out<=s1_gray; b_out<=s1_gray;
                when "001" =>
                    if s4_ebin='1' then
                        r_out<=X"FF"; g_out<=X"FF"; b_out<=X"FF";
                    else
                        r_out<=X"00"; g_out<=X"00"; b_out<=X"00";
                    end if;
                when others =>
                    r_out<=s6_r; g_out<=s6_g; b_out<=s6_b;
            end case;
        end if;
    end process;

end rtl;