create_ndr -name ANALOG_MID_RAIL \
    -width   {Metal2 2.0 Metal3 2.0 Metal4 2.0} \
    -spacing {Metal2 1.0 Metal3 1.0 Metal4 1.0}

set net [get_db nets ua\[0\]]
if {$net != ""} {
    set_db $net .non_default_rule ANALOG_MID_RAIL
}
puts "--- Applying THICK_1V65 NDR to analog mid-rail ---"


# # GF180 default M2/M3 width is ~0.28um. this is forced to 2.0um for high current.
# # spacing is also upped to 1.0um to prevent capacitive coupling noise.
# #create_ndr -name THICK_1V65 \
#  #   -width { Metal2 2.0 Metal3 2.0 Metal4 2.0 } \
#     -spacing { Metal2 1.0 Metal3 1.0 Metal4 1.0 }
# # yosys often collapses `wire net_1v65 = ua[1];` down to just `ua[1]`.
# # thus try looking for the named net first, then fall back to the pad net.
# set analog_net [get_nets -quiet "net_1v65"]
# if { $analog_net == "" } {
#     set analog_net [get_nets "ua[1]"]
# }

# assign_ndr -ndr THICK_1V65 -net $analog_net

puts "--- NDR THICK_1V65 successfully applied ---"