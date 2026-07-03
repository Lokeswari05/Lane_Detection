-- =============================================================
-- rgb2gray.vhd
-- Stage 1: RGB → Grayscale
--
-- Formula: Gray = (77·R + 150·G + 29·B) >> 8
--   Coefficients are ITU-R BT.601 scaled to sum=256.
-- Latency: 1 clock
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity rgb2gray is
    port (
        clk    : in  std_logic;
        -- Control passthrough
        de_in  : in  std_logic;
        hs_in  : in  std_logic;
        vs_in  : in  std_logic;
        -- Input pixel
        r_in   : in  std_logic_vector(7 downto 0);
        g_in   : in  std_logic_vector(7 downto 0);
        b_in   : in  std_logic_vector(7 downto 0);
        -- Original pixel delayed (for overlay later)
        r_d    : out std_logic_vector(7 downto 0);
        g_d    : out std_logic_vector(7 downto 0);
        b_d    : out std_logic_vector(7 downto 0);
        -- Control passthrough
        de_out : out std_logic;
        hs_out : out std_logic;
        vs_out : out std_logic;
        -- Grayscale output
        gray   : out std_logic_vector(7 downto 0)
    );
end rgb2gray;

architecture rtl of rgb2gray is
begin
    process(clk)
        variable acc : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- Weighted sum (max = 77*255+150*255+29*255 = 65280 < 2^17)
            acc := to_unsigned(
                    77  * to_integer(unsigned(r_in)) +
                    150 * to_integer(unsigned(g_in)) +
                    29  * to_integer(unsigned(b_in)),
                    18);
            gray   <= std_logic_vector(acc(15 downto 8)); -- divide by 256
            r_d    <= r_in;
            g_d    <= g_in;
            b_d    <= b_in;
            de_out <= de_in;
            hs_out <= hs_in;
            vs_out <= vs_in;
        end if;
    end process;
end rtl;
