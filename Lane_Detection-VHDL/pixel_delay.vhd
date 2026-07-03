-- pixel_delay.vhd (v2 - uses altshift_taps for efficient RAM-based delay)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pixel_delay is
    generic (
        DELAY_CLOCKS : integer := 9;
        DATA_WIDTH   : integer := 8
    );
    port (
        clk      : in  std_logic;
        data_in  : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        data_out : out std_logic_vector(DATA_WIDTH-1 downto 0)
    );
end pixel_delay;

architecture rtl of pixel_delay is
    type pipe_t is array(0 to DELAY_CLOCKS-1) of
                   std_logic_vector(DATA_WIDTH-1 downto 0);
    signal pipe : pipe_t := (others=>(others=>'0'));
    attribute ramstyle : string;
    attribute ramstyle of pipe : signal is "logic";
begin
    process(clk)
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to DELAY_CLOCKS-1 loop
                pipe(i) <= pipe(i-1);
            end loop;
        end if;
    end process;
    data_out <= pipe(DELAY_CLOCKS-1);
end rtl;