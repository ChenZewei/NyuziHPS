// 
// Copyright 2011-2015 Jeff Bush
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 

`define BITBANG_SDMMC

`include "defines.sv"

module fpga_top(
	input                       clk50,

	// Buttons
	input                       reset_btn,	// KEY[0]

	// Der blinkenlights
	output logic[17:0]          red_led,
	output logic[8:0]           green_led,
	output logic[6:0]           hex0,
	output logic[6:0]           hex1,
	output logic[6:0]           hex2,
	output logic[6:0]           hex3,
	
	// UART
	output                      uart_tx,
	input                       uart_rx,

	// SDRAM	
	/*//JBush SDRam controller 
	output                      dram_clk,   //Clock
	output                      dram_cke,   //Clock Enable
	output                      dram_cs_n,  //Chip Select
	output                      dram_ras_n, //Raw Address Strobe
	output                      dram_cas_n, //Colume Address Strobe
	output                      dram_we_n,  //Write Enable
	output [1:0]                dram_ba,	 //Bank Address
	output [12:0]               dram_addr,  //Address Bus A0-A14
	output [3:0]                dram_dqm,   //Data Mask
	inout [31:0]                dram_dq,    //Data Bus
	*/
	
	//
	output[14:0]                dram_a,  //Address Bus A0-A14
	output [2:0]                dram_ba,	 //Bank Address Bus BA 0 1 2
	output                      dram_cas_n, //Colume Address Strobe
	output                      dram_cke,   //Clock Enable
	output                      dram_clk_p, //Differencial Output Clock
	output                      dram_clk_n, //Differencial Output Clock
	output                      dram_cs_n,  //Chip Select
	output [3:0]                dram_dqm,   //Write Mask Byte Lane
	inout [31:0]                dram_dq,    //Data Bus
	inout [3:0]				   	 dram_dqs_p, //Data Strobe P Byte Lane 0-3
	inout [3:0]				   	 dram_dqs_n, //Data Strobe N Byte Lane 0-3
	output							 dram_odt,   //One-die Termination Enable
	output                      dram_ras_n, //Raw Address Strobe
	output							 dram_resetn,//Reset
	output                      dram_we_n,  //Write Enable
	input 							 dram_rzq,  //ZQ Impedance Calibration
	
	
	
	// VGA
	output [7:0]                vga_r,
	output [7:0]                vga_g,
	output [7:0]                vga_b,
	output                      vga_clk,
	output                      vga_blank_n,
	output                      vga_hs,
	output                      vga_vs,
	output                      vga_sync_n,
	
	// SD card
	output                      sd_clk,
	inout                       sd_cmd,	
	inout[3:0]                  sd_dat,
	
	// PS/2 
	inout                       ps2_clk,
	inout                       ps2_data);

	// We always access the full word width, so hard code these to active (low)
	assign dram_dqm = 4'b0000;

	logic fb_base_update_en;
	logic [31:0] fb_new_base;
	logic frame_toggle;

	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	scalar_t	io_address;		// From nyuzi of nyuzi.v
	wire		io_read_en;		// From nyuzi of nyuzi.v
	scalar_t	io_write_data;		// From nyuzi of nyuzi.v
	wire		io_write_en;		// From nyuzi of nyuzi.v
	logic		pc_event_dram_page_hit;	// From sdram_controller of sdram_controller.v
	logic		pc_event_dram_page_miss;// From sdram_controller of sdram_controller.v
	wire		processor_halt;		// From nyuzi of nyuzi.v
	// End of automatics

	axi4_interface axi_bus_m0();
	axi4_interface axi_bus_m1();
	axi4_interface axi_bus_s0();
	axi4_interface axi_bus_s1();
	logic reset;
	wire[31:0] loader_addr;
	wire[31:0] loader_data;
	wire loader_we;
	logic clk;
	scalar_t io_read_data;
	scalar_t uart_read_data;
	scalar_t sdcard_read_data;
	scalar_t gpio_read_data;
	scalar_t ps2_read_data;
	
	assign clk = clk50;

	/* nyuzi AUTO_TEMPLATE(
		.axi_bus(axi_bus_s0[]),
		);
	*/
	nyuzi #(.RESET_PC(32'hfffee000)) nyuzi(
			.interrupt_req(0),
		/*AUTOINST*/
					       // Interfaces
					       .axi_bus		(axi_bus_s0),	 // Templated
					       // Outputs
					       .processor_halt	(processor_halt),
					       .io_write_en	(io_write_en),
					       .io_read_en	(io_read_en),
					       .io_address	(io_address),
					       .io_write_data	(io_write_data),
					       // Inputs
					       .clk		(clk),
					       .reset		(reset),
					       .io_read_data	(io_read_data));
	
	axi_interconnect axi_interconnect(
		/*AUTOINST*/
					  // Interfaces
					  .axi_bus_m0		(axi_bus_m0.master),
					  .axi_bus_m1		(axi_bus_m1.master),
					  .axi_bus_s0		(axi_bus_s0.slave),
					  .axi_bus_s1		(axi_bus_s1.slave),
					  // Inputs
					  .clk			(clk),
					  .reset		(reset));

	synchronizer reset_synchronizer(
		.clk(clk),
		.reset(0),
		.data_o(reset),
		.data_i(!reset_btn));	// Reset button goes low when pressed

	// Boot ROM.  Execution starts here.
	/* axi_rom AUTO_TEMPLATE(
		.axi_bus(axi_bus_m1.slave),);
	*/
	/*
	axi_rom #(.FILENAME("../../../software/bootrom/boot.hex")) boot_rom(
		/*AUTOINST*/
		/*
									    // Interfaces
									    .axi_bus		(axi_bus_m1.slave), // Templated
									    // Inputs
									    .clk		(clk),
									    .reset		(reset));
		*/
	/* sdram_controller AUTO_TEMPLATE(
		.clk(clk),
		.axi_bus(axi_bus_m0.slave),);
	*/
	
	/*
	sdram_controller #(
			.DATA_WIDTH(32), 
			.ROW_ADDR_WIDTH(13), 
			.COL_ADDR_WIDTH(10),

			// 50 Mhz = 20ns clock.  Each value is clocks of delay minus one.
			// Timing values based on datasheet for A3V64S40ETP SDRAM parts
			// on the DE2-115 board.
			.T_REFRESH(390),          // 64 ms / 8192 rows = 7.8125 uS  
			.T_POWERUP(10000),        // 200 us		
			.T_ROW_PRECHARGE(1),      // 21 ns	
			.T_AUTO_REFRESH_CYCLE(3), // 75 ns
			.T_RAS_CAS_DELAY(1),      // 21 ns	
			.T_CAS_LATENCY(1)		  // 21 ns (2 cycles)
		) sdram_controller(
				   // Interfaces
				   .axi_bus		(axi_bus_m0.slave), // Templated
				   // Outputs
				   .dram_clk		(dram_clk),
				   .dram_cke		(dram_cke),
				   .dram_cs_n		(dram_cs_n),
				   .dram_ras_n		(dram_ras_n),
				   .dram_cas_n		(dram_cas_n),
				   .dram_we_n		(dram_we_n),
				   .dram_ba		(dram_ba[1:0]),
				   .dram_addr		(dram_addr[12:0]),
				   .pc_event_dram_page_miss(pc_event_dram_page_miss),
				   .pc_event_dram_page_hit(pc_event_dram_page_hit),
				   // Inouts
				   .dram_dq		(dram_dq[31:0]),
				   // Inputs
				   .clk			(clk),		 // Templated
				   .reset		(reset));
	*/
	
    fpga_sdram_controller u0 (
        .clk_clk                      (clk),                      //                  clk.clk
        .reset_reset_n                (reset),                //                reset.reset_n
        .memory_mem_a                 (dram_a),                 //               memory.mem_a
        .memory_mem_ba                (dram_ba),                //                     .mem_ba
        .memory_mem_ck                (dram_clk_p),                //                     .mem_ck
        .memory_mem_ck_n              (dram_clk_n),              //                     .mem_ck_n
        .memory_mem_cke               (dram_cke),               //                     .mem_cke
        .memory_mem_cs_n              (dram_cs_n),              //                     .mem_cs_n
        .memory_mem_dm                (),                //                     .mem_dm
        .memory_mem_ras_n             (dram_ras_n),             //                     .mem_ras_n
        .memory_mem_cas_n             (dram_cas_n),             //                     .mem_cas_n
        .memory_mem_we_n              (dram_we_n),              //                     .mem_we_n
        .memory_mem_reset_n           (),           //                     .mem_reset_n
        .memory_mem_dq                (dram_dq),                //                     .mem_dq
        .memory_mem_dqs               (dram_dqs_p),               //                     .mem_dqs
        .memory_mem_dqs_n             (dram_dqs_n),             //                     .mem_dqs_n
        .memory_mem_odt               (dram_odt),               //                     .mem_odt
        .oct_rzqin                    (dram_rzq),                    //                  oct.rzqin
        .axi_translator_slave_awid    (),    // axi_translator_slave.awid
        .axi_translator_slave_awaddr  (axi_bus_m0.slave.m_awaddr),  //                     .awaddr
        .axi_translator_slave_awlen   (axi_bus_m0.slave.m_awlen),   //                     .awlen
        .axi_translator_slave_awsize  (axi_bus_m0.slave.m_awsize),  //                     .awsize
        .axi_translator_slave_awburst (axi_bus_m0.slave. m_awburst), //                     .awburst
        .axi_translator_slave_awlock  (),  //                     .awlock
        .axi_translator_slave_awcache (axi_bus_m0.slave.m_awcache), //                     .awcache
        .axi_translator_slave_awprot  (),  //                     .awprot
        .axi_translator_slave_awvalid (axi_bus_m0.slave.m_awvalid), //                     .awvalid
        .axi_translator_slave_awready (axi_bus_m0.slave.s_awready), //                     .awready
        .axi_translator_slave_wid     (),     //                     .wid
        .axi_translator_slave_wdata   (axi_bus_m0.slave.m_wdata),   //                     .wdata
        .axi_translator_slave_wstrb   (axi_bus_m0.slave.m_wstrb),   //                     .wstrb
        .axi_translator_slave_wlast   (axi_bus_m0.slave. m_wlast),   //                     .wlast
        .axi_translator_slave_wvalid  (axi_bus_m0.slave.m_wvalid),  //                     .wvalid
        .axi_translator_slave_wready  (axi_bus_m0.slave.s_wready),  //                     .wready
        .axi_translator_slave_bid     (),     //                     .bid
        .axi_translator_slave_bresp   (),   //                     .bresp
        .axi_translator_slave_bvalid  (axi_bus_m0.slave.s_bvalid),  //                     .bvalid
        .axi_translator_slave_bready  (axi_bus_m0.slave.m_bready),  //                     .bready
        .axi_translator_slave_arid    (),    //                     .arid
        .axi_translator_slave_araddr  (axi_bus_m0.slave.m_araddr),  //                     .araddr
        .axi_translator_slave_arlen   (axi_bus_m0.slave.m_arlen),   //                     .arlen
        .axi_translator_slave_arsize  (axi_bus_m0.slave.m_arsize),  //                     .arsize
        .axi_translator_slave_arburst (axi_bus_m0.slave.m_arburst), //                     .arburst
        .axi_translator_slave_arlock  (),  //                     .arlock
        .axi_translator_slave_arcache (axi_bus_m0.slave.m_arcache), //                     .arcache
        .axi_translator_slave_arprot  (),  //                     .arprot
        .axi_translator_slave_arvalid (axi_bus_m0.slave.m_arvalid), //                     .arvalid
        .axi_translator_slave_arready (axi_bus_m0.slave.s_arready), //                     .arready
        .axi_translator_slave_rid     (),     //                     .rid
        .axi_translator_slave_rdata   (axi_bus_m0.slave.s_rdata),   //                     .rdata
        .axi_translator_slave_rresp   (),   //                     .rresp
        .axi_translator_slave_rlast   (),   //                     .rlast
        .axi_translator_slave_rvalid  (axi_bus_m0.slave.s_rvalid),  //                     .rvalid
        .axi_translator_slave_rready  (axi_bus_m0.slave.m_rready)   //                     .rready
    );

	vga_controller vga_controller(
	      .axi_bus(axi_bus_s1.master),
		  .*);

`ifdef DEBUG_TRACE
	logic[87:0] capture_data;
	logic capture_enable;
	logic trigger;
	logic[31:0] event_count;
	
	assign capture_data = {};
	assign capture_enable = 1;
	assign trigger = event_count == 120;

	debug_trace #(.CAPTURE_WIDTH_BITS($bits(capture_data)), 
		.CAPTURE_SIZE(128),
		.BAUD_DIVIDE(50000000 / 115200)) debug_trace(.*);

	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
			event_count <= 0;
		else if (capture_enable)
			event_count <= event_count + 1;
	end
`else	
	uart #(.BASE_ADDRESS(24), .BAUD_DIVIDE(50000000 / (115200 * 8))) uart(
		.io_read_data(uart_read_data),
		.*);
`endif

`ifdef BITBANG_SDMMC
	gpio_controller #(.BASE_ADDRESS('h58), .NUM_PINS(6)) gpio_controller(
		.io_read_data(gpio_read_data),
		.gpio_value({sd_clk, sd_cmd, sd_dat}),
		.*);
`else
	spi_controller #(.BASE_ADDRESS('h44)) spi_controller(
		.io_read_data(sdcard_read_data),
		.spi_clk(sd_clk),
		.spi_cs_n(sd_dat[3]),
		.spi_miso(sd_dat[0]),
		.spi_mosi(sd_cmd),
		.*);
`endif

	ps2_controller #(.BASE_ADDRESS('h38)) ps2_controller(
		.io_read_data(ps2_read_data),
		.*);

	assign fb_new_base = io_write_data;
	assign fb_base_update_en = io_write_en && io_address == 'h28;
					  
	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			red_led <= 0;
			green_led <= 0;
			hex0 <= 7'b1111111;
			hex1 <= 7'b1111111;
			hex2 <= 7'b1111111;
			hex3 <= 7'b1111111;
		end
		else
		begin
			if (io_write_en)
			begin
				case (io_address)
					'h00: red_led <= io_write_data[17:0];
					'h04: green_led <= io_write_data[8:0];
					'h08: hex0 <= io_write_data[6:0];
					'h0c: hex1 <= io_write_data[6:0];
					'h10: hex2 <= io_write_data[6:0];
					'h14: hex3 <= io_write_data[6:0];
				endcase
			end
		end
	end

	always_ff @(posedge clk)
	begin
		case (io_address)
			'h18, 'h1c: io_read_data <= uart_read_data;
			'h2c: io_read_data <= frame_toggle;
`ifdef BITBANG_SDMMC
			'h5c: io_read_data <= gpio_read_data;
`else
			'h48, 'h4c: io_read_data <= sdcard_read_data;
`endif

			'h38, 'h3c: io_read_data <= ps2_read_data;
			default: io_read_data <= 0;
		endcase
	end
endmodule

// Local Variables:
// verilog-library-flags:("-y ../core" "-y ../testbench")
// verilog-auto-inst-param-value: t
// End:
