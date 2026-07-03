-- =============================================================
-- ransac_line.vhd  (v3)
-- Stage 5: RANSAC Line Fitting
--
-- Changes from v2:
--   * lanes_separated output signal added.
--     Asserted when right_x - left_x >= MIN_SEP at the current
--     evaluation row (VS rising edge row).  The overlay uses
--     this per-row in real time via the separation logic there.
--   * Core first+last fit unchanged (resource-efficient).
--   * Slope clamped to +/-300 (Q8.8 units), intercept clamped
--     to 0..H_ACTIVE-1.
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity ransac_line is
    port (
        clk            : in  std_logic;
        reset          : in  std_logic;
        vs_in          : in  std_logic;
        blob_valid     : in  std_logic;
        blob_count     : in  integer range 0 to MAX_BLOBS;
        blob_row       : in  integer range 0 to V_ACTIVE-1;
        blob_x         : in  integer range 0 to H_ACTIVE-1;
        blob_x1        : in  integer range 0 to H_ACTIVE-1;
        blob_x2        : in  integer range 0 to H_ACTIVE-1;
        blob_x3        : in  integer range 0 to H_ACTIVE-1;
        blob_x4        : in  integer range 0 to H_ACTIVE-1;
        blob_x5        : in  integer range 0 to H_ACTIVE-1;
        blob_x6        : in  integer range 0 to H_ACTIVE-1;
        blob_x7        : in  integer range 0 to H_ACTIVE-1;
        lane_l_valid   : out std_logic;
        lane_r_valid   : out std_logic;
        lane_l_a       : out integer range -65536 to 65535;
        lane_l_b       : out integer range -65536 to 65535;
        lane_r_a       : out integer range -65536 to 65535;
        lane_r_b       : out integer range -65536 to 65535
    );
end ransac_line;

architecture rtl of ransac_line is

    constant HALF_X : integer := H_ACTIVE / 2;

    -- Per-frame: track first + last blob per side
    signal l_xf, l_xl : integer range 0 to H_ACTIVE-1 := 320;
    signal l_yf, l_yl : integer range 0 to V_ACTIVE-1 := ROI_TOP;
    signal l_got      : std_logic := '0';
    signal l_cnt      : integer range 0 to 255 := 0;

    signal r_xf, r_xl : integer range 0 to H_ACTIVE-1 := 960;
    signal r_yf, r_yl : integer range 0 to V_ACTIVE-1 := ROI_TOP;
    signal r_got      : std_logic := '0';
    signal r_cnt      : integer range 0 to 255 := 0;

    -- Latched for fit computation
    signal fl_xf, fl_xl : integer range 0 to H_ACTIVE-1 := 320;
    signal fl_yf, fl_yl : integer range 0 to V_ACTIVE-1 := ROI_TOP;
    signal fl_cnt       : integer range 0 to 255 := 0;
    signal fr_xf, fr_xl : integer range 0 to H_ACTIVE-1 := 960;
    signal fr_yf, fr_yl : integer range 0 to V_ACTIVE-1 := ROI_TOP;
    signal fr_cnt       : integer range 0 to 255 := 0;

    -- Lane outputs
    signal l_valid : std_logic := '0';
    signal r_valid : std_logic := '0';
    signal l_a     : integer range -65536 to 65535 := 0;
    signal l_b     : integer range -65536 to 65535 := 320;
    signal r_a     : integer range -65536 to 65535 := 0;
    signal r_b     : integer range -65536 to 65535 := 960;

    signal prev_vs  : std_logic := '0';
    signal fit_step : integer range 0 to 7 := 0;

    -- Sequential divider
    signal div_num  : integer range -163840 to 163840 := 0;
    signal div_den  : integer range 0 to V_ACTIVE := 1;
    signal div_res  : integer range -1280 to 1280 := 0;
    signal div_sign : std_logic := '0';
    signal div_cnt  : integer range 0 to 511 := 0;
    signal div_done : std_logic := '0';

begin

    process(clk)
        variable bx  : integer range 0 to H_ACTIVE-1;
        variable by  : integer range 0 to V_ACTIVE-1;
        variable dx, dy : integer;
        variable ca, cb : integer;
        variable num_abs : integer range 0 to 163840;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                l_got<='0'; r_got<='0'; l_cnt<=0; r_cnt<=0;
                l_valid<='0'; r_valid<='0';
                fit_step<=0; prev_vs<='0';
                div_done<='0';
            else
                prev_vs  <= vs_in;
                div_done <= '0';

                -- VS rising: latch blobs, trigger fit
                if vs_in='1' and prev_vs='0' then
                    fl_xf<=l_xf; fl_xl<=l_xl;
                    fl_yf<=l_yf; fl_yl<=l_yl; fl_cnt<=l_cnt;
                    fr_xf<=r_xf; fr_xl<=r_xl;
                    fr_yf<=r_yf; fr_yl<=r_yl; fr_cnt<=r_cnt;
                    l_got<='0'; r_got<='0'; l_cnt<=0; r_cnt<=0;
                    fit_step <= 1;
                end if;

                -- Absorb primary blob (left or right based on x)
                if blob_valid='1' and blob_count>=1 then
                    bx := blob_x; by := blob_row;
                    if bx < HALF_X then
                        if l_got='0' then l_xf<=bx; l_yf<=by; l_got<='1'; end if;
                        l_xl<=bx; l_yl<=by;
                        if l_cnt<255 then l_cnt<=l_cnt+1; end if;
                    else
                        if r_got='0' then r_xf<=bx; r_yf<=by; r_got<='1'; end if;
                        r_xl<=bx; r_yl<=by;
                        if r_cnt<255 then r_cnt<=r_cnt+1; end if;
                    end if;
                end if;

                -- Absorb secondary blob
                if blob_valid='1' and blob_count>=2 then
                    bx := blob_x1; by := blob_row;
                    if bx < HALF_X then
                        l_xl<=bx; l_yl<=by;
                        if l_cnt<255 then l_cnt<=l_cnt+1; end if;
                    else
                        r_xl<=bx; r_yl<=by;
                        if r_cnt<255 then r_cnt<=r_cnt+1; end if;
                    end if;
                end if;

                -- ── Fit state machine ─────────────────────────
                case fit_step is
                    when 0 => null;

                    when 1 =>   -- Setup LEFT slope
                        dy := fl_yl - fl_yf;
                        dx := fl_xl - fl_xf;
                        if dy > 8 and fl_cnt >= 4 then
                            div_num  <= dx * 256;
                            div_den  <= dy;
                            div_cnt  <= 0;
                            div_res  <= 0;
                            if dx < 0 then
                                div_sign <= '1';
                                div_num  <= (-dx) * 256;
                            else
                                div_sign <= '0';
                            end if;
                            fit_step <= 2;
                        else
                            l_a      <= 0;
                            l_b      <= (fl_xf + fl_xl) / 2;
                            if fl_cnt >= 2 then l_valid <= '1'; end if;
                            fit_step <= 4;
                        end if;

                    when 2 =>   -- Sequential divide LEFT
                        num_abs := div_num;
                        if num_abs >= div_den then
                            div_num <= num_abs - div_den;
                            div_res <= div_res + 1;
                        else
                            div_done <= '1';
                            fit_step <= 3;
                        end if;
                        if div_cnt >= 511 then
                            div_done <= '1'; fit_step <= 3;
                        end if;
                        div_cnt <= div_cnt + 1;

                    when 3 =>   -- Apply LEFT result
                        if div_sign = '1' then ca := -div_res;
                        else ca := div_res; end if;
                        if ca >  300 then ca :=  300; end if;
                        if ca < -300 then ca := -300; end if;
                        cb := fl_xf - ca * fl_yf / 256;
                        if cb < 0    then cb := 0;    end if;
                        if cb > H_ACTIVE-1 then cb := H_ACTIVE-1; end if;
                        l_a <= ca; l_b <= cb;
                        l_valid <= '1';
                        fit_step <= 4;

                    when 4 =>   -- Setup RIGHT slope
                        dy := fr_yl - fr_yf;
                        dx := fr_xl - fr_xf;
                        if dy > 8 and fr_cnt >= 4 then
                            div_num  <= dx * 256;
                            div_den  <= dy;
                            div_cnt  <= 0;
                            div_res  <= 0;
                            if dx < 0 then
                                div_sign <= '1';
                                div_num  <= (-dx) * 256;
                            else
                                div_sign <= '0';
                            end if;
                            fit_step <= 5;
                        else
                            r_a      <= 0;
                            r_b      <= (fr_xf + fr_xl) / 2;
                            if fr_cnt >= 2 then r_valid <= '1'; end if;
                            fit_step <= 0;
                        end if;

                    when 5 =>   -- Sequential divide RIGHT
                        num_abs := div_num;
                        if num_abs >= div_den then
                            div_num <= num_abs - div_den;
                            div_res <= div_res + 1;
                        else
                            div_done <= '1'; fit_step <= 6;
                        end if;
                        if div_cnt >= 511 then
                            div_done <= '1'; fit_step <= 6;
                        end if;
                        div_cnt <= div_cnt + 1;

                    when 6 =>   -- Apply RIGHT result
                        if div_sign = '1' then ca := -div_res;
                        else ca := div_res; end if;
                        if ca >  300 then ca :=  300; end if;
                        if ca < -300 then ca := -300; end if;
                        cb := fr_xf - ca * fr_yf / 256;
                        if cb < 0    then cb := 0;    end if;
                        if cb > H_ACTIVE-1 then cb := H_ACTIVE-1; end if;
                        r_a <= ca; r_b <= cb;
                        r_valid <= '1';
                        fit_step <= 0;

                    when others => fit_step <= 0;
                end case;

            end if;
        end if;
    end process;

    lane_l_valid <= l_valid;
    lane_r_valid <= r_valid;
    lane_l_a     <= l_a;
    lane_l_b     <= l_b;
    lane_r_a     <= r_a;
    lane_r_b     <= r_b;

end rtl;
