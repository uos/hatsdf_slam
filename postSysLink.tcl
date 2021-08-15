# author Marcel Flottmann
# date   2020-10-19

# set_property -dict [list CONFIG.M00_HAS_DATA_FIFO {1}] [get_bd_cells axi_ic_zynq_ultra_ps_e_0_S_AXI_HP0_FPD]

set interconnects [get_bd_cells -regexp "axi_ic_.*_S_AXI_.*"]

foreach ic $interconnects {
    set num_si [get_property CONFIG.NUM_SI $ic]
    set num_mi [get_property CONFIG.NUM_MI $ic]

    puts "Set CONFIG.STRATEGY of $ic"
    set config [list CONFIG.STRATEGY {0}]

    for {set num 0} { $num < $num_si } { incr num } {
        puts [format "Set CONFIG.S%02d_HAS_DATA_FIFO of %s" $num $ic]
        lappend config [format "CONFIG.S%02d_HAS_DATA_FIFO" $num] {1}
    }

    for {set num 0} { $num < $num_mi } { incr num } {
        puts [format "Set CONFIG.M%02d_HAS_DATA_FIFO of %s" $num $ic]
        lappend config [format "CONFIG.M%02d_HAS_DATA_FIFO" $num] {1}
    }

    set_property -dict $config $ic
}