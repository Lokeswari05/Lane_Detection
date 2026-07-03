-- =============================================================
-- roi_mask.vhd  (v3)
-- Stage 2: Trapezoid ROI mask  +  White/Yellow color filter
--
-- Changes from v2:
--   * ROI shape changed from horizontal cut to trapezoid.
--     Per-row left/right column bounds are linearly interpolated
--     between (ROI_TOP_L, ROI_TOP_R) at ROI_TOP_ROW and
--     (ROI_BOT_L, ROI_BOT_R) at V_ACTIVE-1.
--
--   * White/Yellow color filter added BEFORE SDCT:
--       white_hit  = (gray_in >= WHITE_THR)
--       yellow_hit = (R >= YEL_R_MIN) AND (G >= YEL_G_MIN)
--                    AND (B <= YEL_B_MAX)
--     gray_out is passed through only when white_hit OR yellow_hit
--     is true AND the pixel is inside the trapezoid; otherwise 0.
--
-- Latency: 1 clock
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity roi_mask is
    port (
        clk        : in  std_logic;
        reset      : in  std_logic;
        de_in      : in  std_logic;
        hs_in      : in  std_logic;
        vs_in      : in  std_logic;
        gray_in    : in  std_logic_vector(7 downto 0);
        r_in       : in  std_logic_vector(7 downto 0);
        g_in       : in  std_logic_vector(7 downto 0);
        b_in       : in  std_logic_vector(7 downto 0);
        de_out     : out std_logic;
        hs_out     : out std_logic;
        vs_out     : out std_logic;
        gray_out   : out std_logic_vector(7 downto 0);
        r_out      : out std_logic_vector(7 downto 0);
        g_out      : out std_logic_vector(7 downto 0);
        b_out      : out std_logic_vector(7 downto 0);
        roi_active : out std_logic;
        row_num    : out integer range 0 to V_ACTIVE-1
    );
end roi_mask;

architecture rtl of roi_mask is

    signal col_cnt  : integer range 0 to H_TOTAL-1 := 0;
    signal row_cnt  : integer range 0 to V_ACTIVE   := 0;
    signal prev_hs  : std_logic := '0';
    signal prev_vs  : std_logic := '0';

    -- Trapezoid column bounds for current row (updated each HS)
    signal trap_l   : integer range 0 to H_ACTIVE-1 := ROI_BOT_L;
    signal trap_r   : integer range 0 to H_ACTIVE-1 := ROI_BOT_R;

    -- Span for interpolation denominator = V_ACTIVE-1 - ROI_TOP_ROW
    constant ROW_SPAN : integer := V_ACTIVE - 1 - ROI_TOP_ROW;   -- = 314

begin

    process(clk)
        variable gv        : integer range 0 to 255;
        variable rv, gvv, bv : integer range 0 to 255;
        variable white_hit : boolean;
        variable yel_hit   : boolean;
        variable in_trap   : boolean;
        variable in_color  : boolean;
        -- Interpolation helpers (fixed-point *ROW_SPAN to avoid division)
        variable t         : integer range 0 to ROW_SPAN;
        variable new_l     : integer range 0 to H_ACTIVE;
        variable new_r     : integer range 0 to H_ACTIVE;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                col_cnt  <= 0;
                row_cnt  <= 0;
                prev_hs  <= '0';
                prev_vs  <= '0';
                trap_l   <= ROI_BOT_L;
                trap_r   <= ROI_BOT_R;
            else
                prev_hs <= hs_in;
                prev_vs <= vs_in;

                -- VS rising: reset row counter
                if vs_in = '1' and prev_vs = '0' then
                    row_cnt <= 0;
                end if;

                -- HS rising: advance row, recompute trapezoid bounds
                if hs_in = '1' and prev_hs = '0' then
                    col_cnt <= 0;
                    if vs_in = '0' then
                        if row_cnt < V_ACTIVE then
                            row_cnt <= row_cnt + 1;
                        end if;
                    end if;

                    -- Linear interpolation of trapezoid column bounds:
                    --   t = clamp(row - ROI_TOP_ROW, 0, ROW_SPAN)
                    --   trap_l = ROI_TOP_L + (ROI_BOT_L - ROI_TOP_L)*t / ROW_SPAN
                    --   trap_r = ROI_TOP_R + (ROI_BOT_R - ROI_TOP_R)*t / ROW_SPAN
                    -- Using integer arithmetic (Quartus synthesises divider)
                    if row_cnt <= ROI_TOP_ROW then
                        trap_l <= ROI_TOP_L;
                        trap_r <= ROI_TOP_R;
                    elsif row_cnt >= V_ACTIVE - 1 then
                        trap_l <= ROI_BOT_L;
                        trap_r <= ROI_BOT_R;
                    else
                        t     := row_cnt - ROI_TOP_ROW;
                        new_l := ROI_TOP_L + (ROI_BOT_L - ROI_TOP_L) * t / ROW_SPAN;
                        new_r := ROI_TOP_R + (ROI_BOT_R - ROI_TOP_R) * t / ROW_SPAN;
                        if new_l < 0             then new_l := 0; end if;
                        if new_l > H_ACTIVE-1    then new_l := H_ACTIVE-1; end if;
                        if new_r < 0             then new_r := 0; end if;
                        if new_r > H_ACTIVE-1    then new_r := H_ACTIVE-1; end if;
                        trap_l <= new_l;
                        trap_r <= new_r;
                    end if;
                end if;

                -- Column counter
                if de_in = '1' then
                    if col_cnt < H_TOTAL-1 then
                        col_cnt <= col_cnt + 1;
                    end if;
                end if;

                -- Pixel evaluation
                gv  := to_integer(unsigned(gray_in));
                rv  := to_integer(unsigned(r_in));
                gvv := to_integer(unsigned(g_in));
                bv  := to_integer(unsigned(b_in));

                -- Trapezoid check
                in_trap := (row_cnt >= ROI_TOP_ROW) and
                           (col_cnt >= trap_l) and
                           (col_cnt <= trap_r);

                -- White pixel check
                white_hit := (gv >= WHITE_THR);

                -- Yellow pixel check (simplified RGB: R high, G mid, B low)
                yel_hit := (rv  >= YEL_R_MIN) and
                           (gvv >= YEL_G_MIN) and
                           (bv  <= YEL_B_MAX);

                in_color := white_hit or yel_hit;

                -- Outputs
                de_out <= de_in;
                hs_out <= hs_in;
                vs_out <= vs_in;
                r_out  <= r_in;
                g_out  <= g_in;
                b_out  <= b_in;

                if row_cnt < V_ACTIVE then
                    row_num <= row_cnt;
                else
                    row_num <= V_ACTIVE - 1;
                end if;

                -- roi_active: inside trapezoid (for SDCT / blob gating)
                if in_trap then
                    roi_active <= '1';
                else
                    roi_active <= '0';
                end if;

                -- gray_out: pass through only white/yellow pixels inside trap
                if in_trap and in_color then
                    gray_out <= gray_in;
                else
                    gray_out <= (others => '0');
                end if;

            end if;
        end if;
    end process;

end rtl;
