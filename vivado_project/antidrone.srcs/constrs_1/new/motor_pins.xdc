# Pan Motor (PMODA의 아래쪽 4개 핀으로 이사: U18, U19, W18, W19)
set_property -dict { PACKAGE_PIN U18   IOSTANDARD LVCMOS33 } [get_ports { pan_out_0[0] }]; 
set_property -dict { PACKAGE_PIN U19   IOSTANDARD LVCMOS33 } [get_ports { pan_out_0[1] }]; 
set_property -dict { PACKAGE_PIN W18   IOSTANDARD LVCMOS33 } [get_ports { pan_out_0[2] }]; 
set_property -dict { PACKAGE_PIN W19   IOSTANDARD LVCMOS33 } [get_ports { pan_out_0[3] }]; 

# Tilt Motor (PMODB 위쪽 4개 핀 유지: 충돌 없으므로 그대로 둡니다)
set_property -dict { PACKAGE_PIN W14   IOSTANDARD LVCMOS33 } [get_ports { tilt_out_0[0] }]; 
set_property -dict { PACKAGE_PIN Y14   IOSTANDARD LVCMOS33 } [get_ports { tilt_out_0[1] }]; 
set_property -dict { PACKAGE_PIN T11   IOSTANDARD LVCMOS33 } [get_ports { tilt_out_0[2] }]; 
set_property -dict { PACKAGE_PIN T10   IOSTANDARD LVCMOS33 } [get_ports { tilt_out_0[3] }];