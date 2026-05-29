puts "=== [1/7] connect ==="
connect

puts "=== [2/7] system reset ==="
targets 1
rst -system
after 3000

puts "=== [3/7] processor reset ==="
targets 2
rst -processor

puts "=== [4/7] ps7_init ==="
source C:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_platform/hw/sdt/ps7_init.tcl
ps7_init

puts "=== [5/7] fpga bitstream ==="
fpga -f C:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_platform/hw/sdt/antidrone_wrapper.bit
ps7_post_config

puts "=== [6/7] download elf ==="
catch {stop}
dow C:/Users/kimse/capstone/antidrone/vitis_workspace/antidrone_app/build/antidrone_app.elf

puts "=== [7/7] run ==="
con
puts "=== Done: antidrone_app running on PYNQ-Z2 ==="
