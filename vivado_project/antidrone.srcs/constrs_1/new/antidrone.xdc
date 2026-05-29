# -------------------------------------------------------------------------
# Pmod JA - LD2450 Radar Connection (UART1 EMIO)
# -------------------------------------------------------------------------
# LD2450 TX (송신) -> Zynq UART1 RX (수신) : Pmod JA Pin 1
set_property -dict { PACKAGE_PIN Y18   IOSTANDARD LVCMOS33 } [get_ports { UART_1_0_rxd }]; 

# LD2450 RX (수신) -> Zynq UART1 TX (송신) : Pmod JA Pin 2
set_property -dict { PACKAGE_PIN Y19   IOSTANDARD LVCMOS33 } [get_ports { UART_1_0_txd }];