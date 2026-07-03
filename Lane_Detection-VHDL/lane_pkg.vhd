-- =============================================================
-- lane_pkg.vhd  (v3)
-- Shared types, constants, and helper functions.
--
-- Pipeline stages:
--  1. RGB -> Grayscale
--  2. ROI mask  (trapezoid)  +  White/Yellow color filter
--  3. SDCT Edge Detection (3-tap H + V)
--  4. Blob Analysis  (min/max run filter, best-run per half)
--  5. RANSAC Line Fitting  (first+last fit)
--  6. Lane Overlay  (separation-gated draw)
--
-- Target : Cyclone V  5CEBA2F17C6
-- Video  : 1280x720p @ 60 Hz  (74.25 MHz pixel clock)
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package lane_pkg is

    -- -------------------------------------------------------
    -- Video timing
    -- -------------------------------------------------------
    constant H_ACTIVE    : integer := 1280;
    constant V_ACTIVE    : integer := 720;
    constant H_TOTAL     : integer := 1650;
    constant V_TOTAL     : integer := 750;

    -- -------------------------------------------------------
    -- ROI trapezoid (bottom ~45% of frame)
    --   Top edge  : row ROI_TOP_ROW,  cols ROI_TOP_L .. ROI_TOP_R
    --   Bot edge  : row V_ACTIVE-1,   cols ROI_BOT_L .. ROI_BOT_R
    --   Per-row col bounds are linearly interpolated in roi_mask.
    -- -------------------------------------------------------
    constant ROI_TOP_ROW : integer := 405;
    constant ROI_TOP_L   : integer := 420;
    constant ROI_TOP_R   : integer := 860;
    constant ROI_BOT_L   : integer := 60;
    constant ROI_BOT_R   : integer := 1220;

    -- Legacy alias (used by blob_analysis, ransac_line row checks)
    constant ROI_TOP     : integer := ROI_TOP_ROW;

    -- -------------------------------------------------------
    -- Color filter thresholds (applied in roi_mask before SDCT)
    --   White  : gray >= WHITE_THR
    --   Yellow : R >= YEL_R_MIN  AND  G >= YEL_G_MIN  AND  B <= YEL_B_MAX
    -- -------------------------------------------------------
    constant WHITE_THR   : integer := 180;
    constant YEL_R_MIN   : integer := 160;
    constant YEL_G_MIN   : integer := 130;
    constant YEL_B_MAX   : integer := 80;

    -- -------------------------------------------------------
    -- SDCT edge threshold
    -- -------------------------------------------------------
    constant EDGE_THR    : integer := 30;

    -- -------------------------------------------------------
    -- Blob parameters
    -- -------------------------------------------------------
    constant MAX_BLOBS   : integer := 8;
    constant MIN_RUN     : integer := 3;   -- minimum run width accepted
    constant MAX_RUN     : integer := 90;  -- maximum run width accepted

    -- -------------------------------------------------------
    -- RANSAC
    -- -------------------------------------------------------
    constant RANSAC_ITER : integer := 16;

    -- -------------------------------------------------------
    -- Overlay separation gate
    --   Lane lines drawn only where right_x - left_x >= MIN_SEP
    -- -------------------------------------------------------
    constant MIN_SEP     : integer := 50;

    -- -------------------------------------------------------
    -- Pipeline latency
    -- -------------------------------------------------------
    constant PIPE_LAT    : integer := 14;

    -- -------------------------------------------------------
    -- Helpers
    -- -------------------------------------------------------
    function clamp8(v : integer) return integer;
    function iabs(v : integer) return integer;

end package lane_pkg;

package body lane_pkg is

    function clamp8(v : integer) return integer is
    begin
        if v < 0 then return 0;
        elsif v > 255 then return 255;
        else return v;
        end if;
    end function;

    function iabs(v : integer) return integer is
    begin
        if v < 0 then return -v;
        else return v;
        end if;
    end function;

end package body lane_pkg;
