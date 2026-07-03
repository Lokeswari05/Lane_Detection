-- =============================================================
-- lane_overlay.vhd  (v3)
-- Stage 6: Lane Overlay Renderer
--
-- Changes from v2:
--   * Separation gate added:
--       Lane lines are drawn only when right_x - left_x >= MIN_SEP.
--       This prevents green and blue lines from intersecting or
--       overlapping near the vanishing point.
--   * When lanes are too close (sep < MIN_SEP) the original pixel
--     colour is passed through unchanged (no overlay at all).
--   * Fill tint also gated by same separation check.
--
-- Line colours (unchanged):
--   Left  lane  → GREEN  (R=0, G=255, B=0)
--   Right lane  → BLUE   (R=0, G=0,   B=255)
--   Fill region → +60 R, +60 G  (yellow tint)
--
-- Latency: 1 clock
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity lane_overlay is
    port (
        clk          : in  std_logic;
        de_in        : in  std_logic;
        hs_in        : in  std_logic;
        vs_in        : in  std_logic;
        r_in         : in  std_logic_vector(7 downto 0);
        g_in         : in  std_logic_vector(7 downto 0);
        b_in         : in  std_logic_vector(7 downto 0);
        roi_in       : in  std_logic;
        row_in       : in  integer range 0 to V_ACTIVE-1;
        lane_l_valid : in  std_logic;
        lane_r_valid : in  std_logic;
        lane_l_a     : in  integer range -65536 to 65535;
        lane_l_b     : in  integer range -65536 to 65535;
        lane_r_a     : in  integer range -65536 to 65535;
        lane_r_b     : in  integer range -65536 to 65535;
        de_out       : out std_logic;
        hs_out       : out std_logic;
        vs_out       : out std_logic;
        r_out        : out std_logic_vector(7 downto 0);
        g_out        : out std_logic_vector(7 downto 0);
        b_out        : out std_logic_vector(7 downto 0)
    );
end lane_overlay;

architecture rtl of lane_overlay is

    constant LINE_W : integer := 5;

    signal col_cnt  : integer range 0 to H_TOTAL-1 := 0;
    signal prev_hs  : std_logic := '0';
    signal prev_vs  : std_logic := '0';

    signal left_x   : integer range -256 to H_ACTIVE+256 := 320;
    signal right_x  : integer range -256 to H_ACTIVE+256 := 960;

    -- Separation flag: set at HS time, used per-pixel
    signal sep_ok   : std_logic := '0';

begin

    process(clk)
        variable ri, gi, bi     : integer range 0 to 255;
        variable lx, rx         : integer range -256 to H_ACTIVE+256;
        variable dist_l, dist_r : integer range 0 to H_ACTIVE+256;
        variable sep            : integer range -H_ACTIVE to H_ACTIVE+256;
        variable on_left        : boolean;
        variable on_right       : boolean;
        variable in_fill        : boolean;
    begin
        if rising_edge(clk) then
            prev_hs <= hs_in;
            prev_vs <= vs_in;

            if vs_in='1' and prev_vs='0' then
                col_cnt <= 0;
            end if;

            -- HS rising: compute left_x, right_x, sep_ok for this row
            if hs_in='1' and prev_hs='0' then
                col_cnt <= 0;

                if lane_l_valid='1' then
                    lx := lane_l_a * row_in / 256 + lane_l_b;
                    if lx < 0          then lx := 0;          end if;
                    if lx > H_ACTIVE-1 then lx := H_ACTIVE-1; end if;
                    left_x <= lx;
                else
                    lx     := H_ACTIVE / 4;
                    left_x <= lx;
                end if;

                if lane_r_valid='1' then
                    rx := lane_r_a * row_in / 256 + lane_r_b;
                    if rx < 0          then rx := 0;          end if;
                    if rx > H_ACTIVE-1 then rx := H_ACTIVE-1; end if;
                    right_x <= rx;
                else
                    rx      := (3 * H_ACTIVE) / 4;
                    right_x <= rx;
                end if;

                -- Separation gate: both valid AND gap >= MIN_SEP
                sep := rx - lx;
                if lane_l_valid='1' and lane_r_valid='1' and sep >= MIN_SEP then
                    sep_ok <= '1';
                else
                    sep_ok <= '0';
                end if;
            end if;

            if de_in='1' then
                if col_cnt < H_TOTAL-1 then col_cnt <= col_cnt+1; end if;
            end if;

            -- Pixel rendering
            ri := to_integer(unsigned(r_in));
            gi := to_integer(unsigned(g_in));
            bi := to_integer(unsigned(b_in));

            if de_in='1' and roi_in='1' and sep_ok='1' then

                if col_cnt >= left_x then
                    dist_l := col_cnt - left_x;
                else
                    dist_l := left_x - col_cnt;
                end if;

                if col_cnt >= right_x then
                    dist_r := col_cnt - right_x;
                else
                    dist_r := right_x - col_cnt;
                end if;

                on_left  := (dist_l <= LINE_W) and (lane_l_valid='1');
                on_right := (dist_r <= LINE_W) and (lane_r_valid='1');
                in_fill  := (col_cnt > left_x  + LINE_W) and
                            (col_cnt < right_x - LINE_W) and
                            (lane_l_valid='1') and (lane_r_valid='1');

                if on_left then
                    ri := 0; gi := 255; bi := 0;        -- GREEN
                elsif on_right then
                    ri := 0; gi := 0;   bi := 255;      -- BLUE
                elsif in_fill then
                    if ri + 60 > 255 then ri := 255; else ri := ri + 60; end if;
                    if gi + 60 > 255 then gi := 255; else gi := gi + 60; end if;
                end if;

            end if;
            -- sep_ok='0' → pass-through (no overlay drawn)

            de_out <= de_in;
            hs_out <= hs_in;
            vs_out <= vs_in;
            r_out  <= std_logic_vector(to_unsigned(ri, 8));
            g_out  <= std_logic_vector(to_unsigned(gi, 8));
            b_out  <= std_logic_vector(to_unsigned(bi, 8));

        end if;
    end process;

end rtl;
