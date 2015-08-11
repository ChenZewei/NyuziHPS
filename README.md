NyuziHPS
========
##NyuziHPS Introduction NyuziHPS 简介
Implementation of the connection of NyuziProcesser(FPGA), SDRAM controller and HPS(ARM Cortex-A9 Dual) with AXI interconnect.  
I have roughly modified the AXI interconnect in the NyuziProcesser project to add more axi master and slave port with simple arbiter.  
NyuziProcesser Project: https://github.com/jbush001/NyuziProcessor  
通过AXI interconnect实现Nyuziprocesser与SDRAM Controller和HPS(ARM Cortex-A9 Dual)的连接。  
我已经粗略的修改了NyuziProcesser中的AXI interconnect来增加更多的axi master和slave端口以及非常简单的仲裁。  
![image](https://github.com/ChenZewei/NyuziHPS/raw/master/pic/environment.png)  

###Project Component 项目组成
Folder core includes NyuziProcesser core and folder fpga includes NyuziProcesser environment like axi interconnect.  
Folder fpga_sdram_controller includes the altera's sdram controller ip core.  
Folder HPS includes the HPS section, which provide axi master port to be connected with.  
文件夹core是NyuziProcesser的核心，文件夹fpga是其环境包括axi interconnect。  
文件夹fpga_sdram_controller包含了altera的sdram controller ip核。  
文件夹HPS包含了硬核部分，提供了用于连接的axi master端口。  

##Project Configuration 项目配置
Assignments->Settings->Libraries add project library ./core.  
Assignments->Settings->Analysis&Synthesis Settings->Verilog HDL Input add macro VENDRO_ALTERA which setting is "1".  
Run Tcl scripts, which named *_pin_assignments.tcl and *_timing.tcl for both HPS and FPGA_SDRAM_Controller, before Filter(Place & Route).  

##SDRAM Configurations SDRAM 配置
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

##Solutions for errors(Continuously Update) 错误解决（持续更新）
Error (169008): Can't turn on open-drain option for differential I/O pin.  
Assignments->Settings->Analysis&Synthesis Settings->More Settings->Auto Open-Drain Pins turn off (on as default).  
将Assignments->Settings->Analysis&Synthesis Settings->More Settings中的Auto Open-Drain Pins选项关闭。  

Error (175006): Could not find path between the DLL and destination DQS Group.  
Clock pin location may be too far away for the ip cores, change the clk location to a nearer location (depends on the I/O bank).  
时钟输入引脚可能离所配置ip核太远了，需要改变时钟输入引脚到更近的位置。  

