NyuziHPS
========
##NyuziHPS Introduction
Implementation of the connection of NyuziProcesser(FPGA), SDRAM controller and ARM Cortex-A9 Dual(HPS) with AXI interconnect.  
NyuziProcesser Project: https://github.com/jbush001/NyuziProcessor  

##Project Configuration
Assignments->Settings->Libraries add project library ./core.  
Assignments->Settings->Analysis&Synthesis Settings->Verilog HDL Input add macro VENDRO_ALTERA which setting is "1".  
Run Tcl scripts, which named *_pin_assignments.tcl and *_timing.tcl for both HPS and FPGA_SDRAM_Controller, before Filter(Place & Route).  

##SDRAM Configurations
configurations for both HPS and FPGA SDRAMs.  

###PHY Settings  
Memory clock frequency 	`350` is ok.  
###Memory Parameters
Total interface width 	`32`  
Row address width 		`13`  
colume address width 	`10`  
bank-address width 		`3`  
Enable DM pins 			`clicked`  
DQS# Enable 			`clicked`  

##Solutions for errors(Continuously Update)
Error (169008): Can't turn on open-drain option for differential I/O pin.  
Assignments->Settings->Analysis&Synthesis Settings->More Settings->Auto Open-Drain Pins turn off (on as default).  

Error (175006): Could not find path between the DLL and destination DQS Group.  
Clock pin location may be too far away for the ip cores, change the clk location to a nearer location (depends on the I/O bank).  

