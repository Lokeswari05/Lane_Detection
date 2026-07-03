-- line_buffer_7tap.vhd (v7 - correct ramstyle attribute declaration)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity line_buffer_7tap is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        de_in   : in  std_logic;
        hs_in   : in  std_logic;
        vs_in   : in  std_logic;
        data_in : in  std_logic_vector(7 downto 0);
        de_out  : out std_logic;
        hs_out  : out std_logic;
        vs_out  : out std_logic;
        tap0    : out std_logic_vector(7 downto 0);
        tap1    : out std_logic_vector(7 downto 0);
        tap2    : out std_logic_vector(7 downto 0);
        tap3    : out std_logic_vector(7 downto 0);
        tap4    : out std_logic_vector(7 downto 0);
        tap5    : out std_logic_vector(7 downto 0);
        tap6    : out std_logic_vector(7 downto 0)
    );
end line_buffer_7tap;

architecture rtl of line_buffer_7tap is

    subtype byte_t is std_logic_vector(7 downto 0);
    type line_ram_t is array(0 to H_ACTIVE-1) of byte_t;

    -- Attribute must be declared BEFORE use in VHDL-93
    attribute ramstyle : string;

    signal R0 : line_ram_t;
    signal R1 : line_ram_t;
    signal R2 : line_ram_t;
    signal R3 : line_ram_t;
    signal R4 : line_ram_t;
    signal R5 : line_ram_t;

    attribute ramstyle of R0 : signal is "M10K";
    attribute ramstyle of R1 : signal is "M10K";
    attribute ramstyle of R2 : signal is "M10K";
    attribute ramstyle of R3 : signal is "M10K";
    attribute ramstyle of R4 : signal is "M10K";
    attribute ramstyle of R5 : signal is "M10K";

    -- Column counter
    signal col     : integer range 0 to H_ACTIVE-1 := 0;
    -- One-hot bank select (rotates left each new line)
    signal wr_sel  : std_logic_vector(5 downto 0) := "000001";
    signal prev_hs : std_logic := '0';

    -- Registered read outputs from all 6 RAMs
    signal q0,q1,q2,q3,q4,q5 : byte_t := (others=>'0');
    -- Output tap registers
    signal t0,t1,t2,t3,t4,t5 : byte_t := (others=>'0');

begin

    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                col     <= 0;
                wr_sel  <= "000001";
                prev_hs <= '0';
            else
                prev_hs <= hs_in;

                -- New line: rotate bank select left by 1
                if hs_in = '1' and prev_hs = '0' then
                    col    <= 0;
                    wr_sel <= wr_sel(4 downto 0) & wr_sel(5);
                end if;

                if de_in = '1' then
                    -- Write to selected RAM only (one-hot)
                    if wr_sel(0) = '1' then R0(col) <= data_in; end if;
                    if wr_sel(1) = '1' then R1(col) <= data_in; end if;
                    if wr_sel(2) = '1' then R2(col) <= data_in; end if;
                    if wr_sel(3) = '1' then R3(col) <= data_in; end if;
                    if wr_sel(4) = '1' then R4(col) <= data_in; end if;
                    if wr_sel(5) = '1' then R5(col) <= data_in; end if;

                    -- Read ALL RAMs at same column (registered M10K read)
                    q0 <= R0(col);
                    q1 <= R1(col);
                    q2 <= R2(col);
                    q3 <= R3(col);
                    q4 <= R4(col);
                    q5 <= R5(col);

                    -- Assign taps: newest=data_in, then q of previous banks
                    -- wr_sel tells which RAM is CURRENTLY being written
                    -- Previous lines are in the other RAMs in rotation order
                    if    wr_sel(0)='1' then
                        t0<=data_in; t1<=q5; t2<=q4; t3<=q3; t4<=q2; t5<=q1;
                    elsif wr_sel(1)='1' then
                        t0<=data_in; t1<=q0; t2<=q5; t3<=q4; t4<=q3; t5<=q2;
                    elsif wr_sel(2)='1' then
                        t0<=data_in; t1<=q1; t2<=q0; t3<=q5; t4<=q4; t5<=q3;
                    elsif wr_sel(3)='1' then
                        t0<=data_in; t1<=q2; t2<=q1; t3<=q0; t4<=q5; t5<=q4;
                    elsif wr_sel(4)='1' then
                        t0<=data_in; t1<=q3; t2<=q2; t3<=q1; t4<=q0; t5<=q5;
                    else
                        t0<=data_in; t1<=q4; t2<=q3; t3<=q2; t4<=q1; t5<=q0;
                    end if;

                    if col < H_ACTIVE-1 then col <= col+1; end if;
                end if;

                de_out <= de_in;
                hs_out <= hs_in;
                vs_out <= vs_in;
            end if;
        end if;
    end process;

    tap0 <= t0;
    tap1 <= t1;
    tap2 <= t2;
    tap3 <= t3;
    tap4 <= t4;
    tap5 <= t5;
    tap6 <= t5;  -- tap6 = same as tap5 (5 lines ago)

end rtl;