set project [lindex $argv 0]
if {$project eq ""} {
  error "usage: check_synth.tcl <vivado-project>"
}

open_project $project
open_run synth_1

set all_clocks [get_clocks -quiet]
puts "Timing clocks found: [llength $all_clocks]"
foreach clock $all_clocks {
  puts "Clock: [get_property NAME $clock] ([get_property PERIOD $clock] ns), generated=[get_property IS_GENERATED $clock]"
}

set input_clock {}
foreach clock $all_clocks {
  set period [get_property PERIOD $clock]
  if {![get_property IS_GENERATED $clock] && [expr {abs($period - 83.333)}] < 0.001} {
    lappend input_clock $clock
  }
}
if {[llength $input_clock] != 1} {
  error "expected exactly one 12 MHz primary timing clock"
}

set generated_clocks [get_clocks -quiet -filter {IS_GENERATED == 1}]
if {[llength $generated_clocks] == 0} {
  error "no generated clock was derived from the MMCM"
}

set found_100mhz 0
foreach clock $generated_clocks {
  set period [get_property PERIOD $clock]
  if {[expr {abs($period - 10.0)}] < 0.001} {
    set found_100mhz 1
  }
}
if {!$found_100mhz} {
  error "no 100 MHz generated clock was found"
}

set reset_exceptions [report_exceptions -from [get_ports ext_reset] -return_string]
puts $reset_exceptions
if {[string first "false" $reset_exceptions] < 0} {
  error "ext_reset false-path exception is missing"
}

puts "Clock/reset synthesis checks passed"
puts "Primary clock: [get_property NAME $input_clock] ([get_property PERIOD $input_clock] ns)"
foreach clock $generated_clocks {
  puts "Generated clock: [get_property NAME $clock] ([get_property PERIOD $clock] ns)"
}

close_project
