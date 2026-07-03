-- =============================================================
-- blob_analysis.vhd  (v3)
-- Stage 4: Binary Blob Analysis
--
-- Changes from v2:
--   * MAX_RUN filter added: runs wider than MAX_RUN pixels
--     (signs, barriers, guardrails) are rejected.
--   * Best-run selection per half: instead of taking the FIRST
--     qualifying run, the LONGEST run within MIN_RUN..MAX_RUN
--     is chosen as the lane centroid for that half.
--   * Unused blob_x2..blob_x7 outputs remain tied to 0.
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity blob_analysis is
    port (
        clk          : in  std_logic;
        reset        : in  std_logic;
        de_in        : in  std_logic;
        hs_in        : in  std_logic;
        vs_in        : in  std_logic;
        roi_in       : in  std_logic;
        edge_bin     : in  std_logic;
        row_in       : in  integer range 0 to V_ACTIVE-1;
        de_out       : out std_logic;
        hs_out       : out std_logic;
        vs_out       : out std_logic;
        blob_valid   : out std_logic;
        blob_count   : out integer range 0 to MAX_BLOBS;
        blob_row     : out integer range 0 to V_ACTIVE-1;
        blob_x       : out integer range 0 to H_ACTIVE-1;
        blob_x1      : out integer range 0 to H_ACTIVE-1;
        blob_x2      : out integer range 0 to H_ACTIVE-1;
        blob_x3      : out integer range 0 to H_ACTIVE-1;
        blob_x4      : out integer range 0 to H_ACTIVE-1;
        blob_x5      : out integer range 0 to H_ACTIVE-1;
        blob_x6      : out integer range 0 to H_ACTIVE-1;
        blob_x7      : out integer range 0 to H_ACTIVE-1
    );
end blob_analysis;

architecture rtl of blob_analysis is

    constant HALF_X : integer := H_ACTIVE / 2;

    -- ── Left half run tracking ────────────────────────────────
    signal l_in_run    : std_logic := '0';
    signal l_run_start : integer range 0 to H_ACTIVE-1 := 0;
    signal l_run_len   : integer range 0 to H_ACTIVE   := 0;
    signal l_centroid  : integer range 0 to H_ACTIVE-1 := 0;
    signal l_best_len  : integer range 0 to H_ACTIVE   := 0;
    signal l_found     : std_logic := '0';

    -- ── Right half run tracking ───────────────────────────────
    signal r_in_run    : std_logic := '0';
    signal r_run_start : integer range 0 to H_ACTIVE-1 := 0;
    signal r_run_len   : integer range 0 to H_ACTIVE   := 0;
    signal r_centroid  : integer range 0 to H_ACTIVE-1 := 0;
    signal r_best_len  : integer range 0 to H_ACTIVE   := 0;
    signal r_found     : std_logic := '0';

    signal col_cnt  : integer range 0 to H_ACTIVE   := 0;
    signal prev_hs  : std_logic := '0';
    signal prev_de  : std_logic := '0';

    -- Output registers
    signal bv_r   : std_logic := '0';
    signal br_r   : integer range 0 to V_ACTIVE-1 := 0;
    signal bx_r   : integer range 0 to H_ACTIVE-1 := 0;
    signal bx1_r  : integer range 0 to H_ACTIVE-1 := 0;
    signal bcnt_r : integer range 0 to MAX_BLOBS  := 0;

    -- Helper: close a run and update best centroid if qualifying
    -- Implemented inline in the process below (VHDL-93 compatible)

begin

    process(clk)
        -- Temp variables for run-close logic
        variable close_l  : boolean;
        variable close_r  : boolean;
        variable cand_cx  : integer range 0 to H_ACTIVE-1;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                l_in_run <= '0'; r_in_run <= '0';
                col_cnt  <= 0;   bv_r     <= '0';
                prev_hs  <= '0'; prev_de  <= '0';
            else
                prev_hs <= hs_in;
                prev_de <= de_in;
                bv_r    <= '0';

                -- ── New line: reset run state ─────────────────
                if hs_in = '1' and prev_hs = '0' then
                    col_cnt    <= 0;
                    l_in_run   <= '0'; l_run_len <= 0;
                    l_best_len <= 0;   l_found   <= '0';
                    r_in_run   <= '0'; r_run_len <= 0;
                    r_best_len <= 0;   r_found   <= '0';
                end if;

                if de_in = '1' then
                    if col_cnt < H_ACTIVE-1 then
                        col_cnt <= col_cnt + 1;
                    end if;

                    close_l := false;
                    close_r := false;

                    if roi_in = '1' and edge_bin = '1' then
                        -- ── Accumulate runs ───────────────────
                        if col_cnt < HALF_X then
                            if l_in_run = '0' then
                                l_in_run    <= '1';
                                l_run_start <= col_cnt;
                                l_run_len   <= 1;
                            else
                                if l_run_len < H_ACTIVE then
                                    l_run_len <= l_run_len + 1;
                                end if;
                            end if;
                        else
                            if r_in_run = '0' then
                                r_in_run    <= '1';
                                r_run_start <= col_cnt;
                                r_run_len   <= 1;
                            else
                                if r_run_len < H_ACTIVE then
                                    r_run_len <= r_run_len + 1;
                                end if;
                            end if;
                        end if;
                    else
                        -- ── Close runs on gap ─────────────────
                        if l_in_run = '1' then
                            l_in_run <= '0';
                            close_l  := true;
                        end if;
                        if r_in_run = '1' then
                            r_in_run <= '0';
                            close_r  := true;
                        end if;
                    end if;

                    -- ── Evaluate closed LEFT run ──────────────
                    if close_l then
                        if l_run_len >= MIN_RUN and l_run_len <= MAX_RUN then
                            if l_run_len > l_best_len then
                                cand_cx    := l_run_start + l_run_len / 2;
                                l_centroid <= cand_cx;
                                l_best_len <= l_run_len;
                                l_found    <= '1';
                            end if;
                        end if;
                        l_run_len <= 0;
                    end if;

                    -- ── Evaluate closed RIGHT run ─────────────
                    if close_r then
                        if r_run_len >= MIN_RUN and r_run_len <= MAX_RUN then
                            if r_run_len > r_best_len then
                                cand_cx    := r_run_start + r_run_len / 2;
                                r_centroid <= cand_cx;
                                r_best_len <= r_run_len;
                                r_found    <= '1';
                            end if;
                        end if;
                        r_run_len <= 0;
                    end if;

                end if; -- de_in

                -- ── End of active line: close open runs, publish ──
                if de_in = '0' and prev_de = '1' then

                    if l_in_run = '1' then
                        l_in_run <= '0';
                        if l_run_len >= MIN_RUN and l_run_len <= MAX_RUN then
                            if l_run_len > l_best_len then
                                l_centroid <= l_run_start + l_run_len / 2;
                                l_best_len <= l_run_len;
                                l_found    <= '1';
                            end if;
                        end if;
                    end if;

                    if r_in_run = '1' then
                        r_in_run <= '0';
                        if r_run_len >= MIN_RUN and r_run_len <= MAX_RUN then
                            if r_run_len > r_best_len then
                                r_centroid <= r_run_start + r_run_len / 2;
                                r_best_len <= r_run_len;
                                r_found    <= '1';
                            end if;
                        end if;
                    end if;

                    -- Publish
                    br_r <= row_in;
                    bv_r <= '1';

                    if l_found = '1' and r_found = '1' then
                        bx_r   <= l_centroid;
                        bx1_r  <= r_centroid;
                        bcnt_r <= 2;
                    elsif l_found = '1' then
                        bx_r   <= l_centroid;
                        bx1_r  <= 0;
                        bcnt_r <= 1;
                    elsif r_found = '1' then
                        bx_r   <= r_centroid;
                        bx1_r  <= 0;
                        bcnt_r <= 1;
                    else
                        bx_r   <= 0;
                        bx1_r  <= 0;
                        bcnt_r <= 0;
                    end if;

                end if;

                de_out <= de_in;
                hs_out <= hs_in;
                vs_out <= vs_in;

            end if;
        end if;
    end process;

    blob_valid <= bv_r;
    blob_count <= bcnt_r;
    blob_row   <= br_r;
    blob_x     <= bx_r;
    blob_x1    <= bx1_r;
    blob_x2    <= 0;
    blob_x3    <= 0;
    blob_x4    <= 0;
    blob_x5    <= 0;
    blob_x6    <= 0;
    blob_x7    <= 0;

end rtl;
