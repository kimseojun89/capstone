# recreate_platform.tcl
# Deletes and recreates antidrone_platform from the updated XSA.
# Run with: xsct recreate_platform.tcl

set ws   "C:/Users/kimse/capstone/antidrone/vitis_workspace"
set xsa  "C:/Users/kimse/capstone/vivado_project/antidrone_wrapper.xsa"
set pname "antidrone_platform"

puts "=== Setting workspace: $ws ==="
setws $ws

# Remove old platform if it exists
set existing [platform list]
if {[lsearch $existing $pname] >= 0} {
    puts "=== Removing old platform: $pname ==="
    platform remove $pname
}

puts "=== Creating platform: $pname from $xsa ==="
platform create \
    -name       $pname \
    -hw         $xsa \
    -proc       ps7_cortexa9_0 \
    -os         standalone \
    -fsbl-target psu_cortexa53_0

# Vitis Unified 2023.2 on Zynq-7000 uses ps7_cortexa9_0
# Re-create with correct proc
platform remove $pname

platform create \
    -name $pname \
    -hw   $xsa

# Add standalone domain on ps7_cortexa9_0
domain create \
    -name      standalone_ps7_cortexa9_0 \
    -os        standalone \
    -proc      ps7_cortexa9_0 \
    -arch      32-bit

platform write
puts "=== Building platform ==="
platform generate

puts "=== Done: $pname built successfully ==="
