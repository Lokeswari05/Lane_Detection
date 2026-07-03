-- sdct_edge.vhd (v3 - absolute minimum logic)
-- Simple 3-tap horizontal + vertical gradient, no division, no multiply
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.lane_pkg.all;

entity sdct_edge is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        de_in   : in  std_logic;
        hs_in   : in  std_logic;
        vs_in   : in  std_logic;
        roi_in  : in  std_logic;
        tap0    : in  std_logic_vector(7 downto 0);
        tap1    : in  std_logic_vector(7 downto 0);
        tap2    : in  std_logic_vector(7 downto 0);
        tap3    : in  std_logic_vector(7 downto 0);
        tap4    : in  std_logic_vector(7 downto 0);
        tap5    : in  std_logic_vector(7 downto 0);
        tap6    : in  std_logic_vector(7 downto 0);
        de_out  : out std_logic;
        hs_out  : out std_logic;
        vs_out  : out std_logic;
        roi_out : out std_logic;
        edge_bin: out std_logic;
        edge_mag: out std_logic_vector(7 downto 0)
    );
end sdct_edge;

architecture rtl of sdct_edge is
    -- 3-tap horizontal shift register on center row
    signal s0,s1,s2 : std_logic_vector(7 downto 0) := (others=>'0');
    -- 1-clock delay for control
    signal de_r,hs_r,vs_r,roi_r : std_logic := '0';
begin
    process(clk)
        variable h,v,m : integer range 0 to 511;
        variable t0,t3,t5 : integer range 0 to 255;
    begin
        if rising_edge(clk) then
            -- Shift 3-tap horizontal register on center row (tap3)
            s2 <= s1; s1 <= s0; s0 <= tap3;

            -- 1-clock delay for control
            de_r <= de_in; hs_r <= hs_in;
            vs_r <= vs_in; roi_r <= roi_in;

            t0 := to_integer(unsigned(s2)); -- left pixel
            t3 := to_integer(unsigned(s0)); -- right pixel
            t5 := to_integer(unsigned(tap0)); -- top row
            
            -- Horizontal gradient: right - left
            if t3 > t0 then h := t3 - t0; else h := t0 - t3; end if;
            -- Vertical gradient: new row - old row (tap0 newest, tap5 oldest)
            v := to_integer(unsigned(tap0));
            if v > to_integer(unsigned(tap5)) then
                v := v - to_integer(unsigned(tap5));
            else
                v := to_integer(unsigned(tap5)) - v;
            end if;

            m := h + v;
            if m > 255 then m := 255; end if;

            edge_mag <= std_logic_vector(to_unsigned(m, 8));

            if roi_r='1' and m > EDGE_THR then
                edge_bin <= '1';
            else
                edge_bin <= '0';
            end if;

            de_out<=de_r; hs_out<=hs_r; vs_out<=vs_r; roi_out<=roi_r;
        end if;
    end process;
end rtl;