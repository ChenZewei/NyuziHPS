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


`include "defines.sv"

//
// This routes AXI transactions between two masters and two slaves
// mapped into different regions of a common address space.
//

module axi_interconnect(
	input					clk,
	input					reset,

	// Master Interface 0 (address 0x00000000 - 0xfffedfff)
	// (This interface acts as a master and controls an externally
	// connected slave)
	axi4_interface.master    axi_bus_m0,

	// Master Interface 1 (address 0xfffee000 - 0xfffeffff) 
	axi4_interface.master   axi_bus_m1,

	// Slave Interface 0 (CPU/L2 cache)
	// This interface acts as a slave and is controlled by an externally
	// connected master
	axi4_interface.slave    axi_bus_s0,

	// Slave Interface 1 (HPS connecter, read/write)
	axi4_interface.slave    axi_bus_s1,
	// Slave Interface 2 (Display Controller, read only)
	axi4_interface.slave    axi_bus_s2);

	//localparam M1_BASE_ADDRESS = 32'hfffee000;
	
	localparam M1_BASE_ADDRESS = 32'h40000000;// 1G FPGA SDRAM(address 0x00000000 - 0x3fffffff) else for other usages
	
	typedef enum {
		STATE_ARBITRATE,
		STATE_ISSUE_ADDRESS,
		STATE_ACTIVE_BURST
	} burst_state_t;

	//
	// Write handling. Only slave interface 0 does writes.
	// XXX I don't explicitly handle the response in the state machine, but it
	// works because everything is in the correct state when the transaction is finished.
	// This could introduce a subtle bug if the behavior of the core changed.
	//
	burst_state_t write_state;
	logic[31:0] write_burst_address;
	logic[7:0] write_burst_length;	// Like axi_awlen, this is number of transfers minus 1
	logic write_master_select;
	logic write_slave_select;

	// Since only slave interface 0 supports writes, we can just hard wire these.
	/*assign axi_bus_m0.m_awaddr = write_burst_address;
	assign axi_bus_m0.m_awlen = write_burst_length;
	assign axi_bus_m0.m_wdata = axi_bus_s0.m_wdata;
	assign axi_bus_m0.m_wlast = axi_bus_s0.m_wlast;
	assign axi_bus_m0.m_bready = axi_bus_s0.m_bready;
	assign axi_bus_m0.m_wstrb = axi_bus_s0.m_wstrb;
	assign axi_bus_m0.m_awburst = axi_bus_s0.m_awburst;
	assign axi_bus_m0.m_awsize = axi_bus_s0.m_awsize;
	assign axi_bus_m1.m_awaddr = write_burst_address - M1_BASE_ADDRESS;
	assign axi_bus_m1.m_awlen = write_burst_length;
	assign axi_bus_m1.m_wdata = axi_bus_s0.m_wdata;
	assign axi_bus_m1.m_wlast = axi_bus_s0.m_wlast;
	assign axi_bus_m1.m_bready = axi_bus_s0.m_bready;
	assign axi_bus_m1.m_wstrb = axi_bus_s0.m_wstrb;
	assign axi_bus_m1.m_awburst = axi_bus_s0.m_awburst;
	assign axi_bus_m1.m_awsize = axi_bus_s0.m_awsize;*/
	
	always_comb
	begin
		if(write_slave_select == 0)
		begin
			// Slave interface 0 is selected
			axi_bus_m0.m_awaddr = write_burst_address;
			axi_bus_m0.m_awlen = write_burst_length;
			axi_bus_m0.m_wdata = axi_bus_s0.m_wdata;
			axi_bus_m0.m_wlast = axi_bus_s0.m_wlast;
			axi_bus_m0.m_bready = axi_bus_s0.m_bready;
			axi_bus_m0.m_wstrb = axi_bus_s0.m_wstrb;
			axi_bus_m0.m_awburst = axi_bus_s0.m_awburst;
			axi_bus_m0.m_awsize = axi_bus_s0.m_awsize;
			axi_bus_m1.m_awaddr = write_burst_address - M1_BASE_ADDRESS;
			axi_bus_m1.m_awlen = write_burst_length;
			axi_bus_m1.m_wdata = axi_bus_s0.m_wdata;
			axi_bus_m1.m_wlast = axi_bus_s0.m_wlast;
			axi_bus_m1.m_bready = axi_bus_s0.m_bready;
			axi_bus_m1.m_wstrb = axi_bus_s0.m_wstrb;
			axi_bus_m1.m_awburst = axi_bus_s0.m_awburst;
			axi_bus_m1.m_awsize = axi_bus_s0.m_awsize;
		end
		else
		begin
			// Slave interface 1 is selected
			axi_bus_m0.m_awaddr = write_burst_address;
			axi_bus_m0.m_awlen = write_burst_length;
			axi_bus_m0.m_wdata = axi_bus_s1.m_wdata;
			axi_bus_m0.m_wlast = axi_bus_s1.m_wlast;
			axi_bus_m0.m_bready = axi_bus_s1.m_bready;
			axi_bus_m0.m_wstrb = axi_bus_s1.m_wstrb;
			axi_bus_m0.m_awburst = axi_bus_s1.m_awburst;
			axi_bus_m0.m_awsize = axi_bus_s1.m_awsize;
			axi_bus_m1.m_awaddr = write_burst_address - M1_BASE_ADDRESS;
			axi_bus_m1.m_awlen = write_burst_length;
			axi_bus_m1.m_wdata = axi_bus_s1.m_wdata;
			axi_bus_m1.m_wlast = axi_bus_s1.m_wlast;
			axi_bus_m1.m_bready = axi_bus_s1.m_bready;
			axi_bus_m1.m_wstrb = axi_bus_s1.m_wstrb;
			axi_bus_m1.m_awburst = axi_bus_s1.m_awburst;
			axi_bus_m1.m_awsize = axi_bus_s1.m_awsize;
		end
	end
	
	assign axi_bus_m0.m_awvalid = write_master_select == 0 && write_state == STATE_ISSUE_ADDRESS;
	assign axi_bus_m1.m_awvalid = write_master_select == 1 && write_state == STATE_ISSUE_ADDRESS;
	
	/*always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			write_state <= STATE_ARBITRATE;
			//AUTORESET
			// Beginning of autoreset for uninitialized flops
			write_burst_address <= 32'h0;
			write_burst_length <= 8'h0;
			write_master_select <= 1'h0;
			// End of automatics
		end
		else if (write_state == STATE_ACTIVE_BURST)
		begin
			// Burst is active.  Check to see when it is finished.
			if (axi_bus_s0.s_wready && axi_bus_s0.m_wvalid)
			begin
				write_burst_length <= write_burst_length - 8'd1;
				if (write_burst_length == 0)
					write_state <= STATE_ARBITRATE;
			end
		end
		else if (write_state == STATE_ISSUE_ADDRESS)
		begin
			// Wait for the slave to accept the address and length
			if (axi_bus_s0.s_awready)
				write_state <= STATE_ACTIVE_BURST;
		end
		else if (axi_bus_s0.m_awvalid)
		begin
			// Start a new write transaction
			write_master_select <=  axi_bus_s0.m_awaddr >= M1_BASE_ADDRESS;
			write_burst_address <= axi_bus_s0.m_awaddr;
			write_burst_length <= axi_bus_s0.m_awlen;
			write_state <= STATE_ISSUE_ADDRESS;
		end
	end*/
	
	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			write_state <= STATE_ARBITRATE;
			/*AUTORESET*/
			// Beginning of autoreset for uninitialized flops
			write_burst_address <= 32'h0;
			write_burst_length <= 8'h0;
			write_master_select <= 1'h0;
			write_slave_select <= 1'h0;
			// End of automatics
		end
		else if (write_state == STATE_ACTIVE_BURST)
		begin
			// Burst is active.  Check to see when it is finished.
			if (axi_bus_s0.s_wready && axi_bus_s0.m_wvalid)
			begin
				write_burst_length <= write_burst_length - 8'd1;
				if (write_burst_length == 0)
					write_state <= STATE_ARBITRATE;
			end
		end
		else if (write_state == STATE_ISSUE_ADDRESS)
		begin
			// Wait for the slave to accept the address and length
			if (axi_bus_s0.s_awready)
				write_state <= STATE_ACTIVE_BURST;
		end
		else if (axi_bus_s0.m_awvalid)
		begin
			// Start a new write transaction
			write_master_select <=  axi_bus_s0.m_awaddr >= M1_BASE_ADDRESS;
			write_slave_select <= 1'h0;
			write_burst_address <= axi_bus_s0.m_awaddr;
			write_burst_length <= axi_bus_s0.m_awlen;
			write_state <= STATE_ISSUE_ADDRESS;
		end
		else if (axi_bus_s1.m_awvalid)
		begin
			// Start a new write transaction
			write_master_select <=  axi_bus_s1.m_awaddr >= M1_BASE_ADDRESS;
			write_slave_select <= 1'h1;
			write_burst_address <= axi_bus_s1.m_awaddr;
			write_burst_length <= axi_bus_s1.m_awlen;
			write_state <= STATE_ISSUE_ADDRESS;
		end
	end
	
	always_comb
	begin
		if (write_master_select == 0)
		begin
			// Master Interface 0 is selected
			axi_bus_m0.m_wvalid = (axi_bus_s0.m_wvalid || axi_bus_s0.m_wvalid) && write_state == STATE_ACTIVE_BURST;
			axi_bus_m1.m_wvalid = 0;
			axi_bus_s0.s_awready = axi_bus_m0.s_awready && write_state == STATE_ISSUE_ADDRESS && write_slave_select == 0;
			axi_bus_s0.s_wready = axi_bus_m0.s_wready && write_state == STATE_ACTIVE_BURST && write_slave_select == 0;
			axi_bus_s0.s_bvalid = axi_bus_m0.s_bvalid && write_slave_select == 0;
			axi_bus_s1.s_awready = axi_bus_m0.s_awready && write_state == STATE_ISSUE_ADDRESS && write_slave_select == 1;
			axi_bus_s1.s_wready = axi_bus_m0.s_wready && write_state == STATE_ACTIVE_BURST && write_slave_select == 1;
			axi_bus_s1.s_bvalid = axi_bus_m0.s_bvalid && write_slave_select == 1;
		end
		else
		begin
			// Master interface 1 is selected
			axi_bus_m0.m_wvalid = 0;
			axi_bus_m1.m_wvalid = (axi_bus_s0.m_wvalid || axi_bus_s0.m_wvalid) && write_state == STATE_ACTIVE_BURST;
			axi_bus_s0.s_awready = axi_bus_m1.s_awready && write_state == STATE_ISSUE_ADDRESS && write_slave_select == 0;
			axi_bus_s0.s_wready = axi_bus_m1.s_wready && write_state == STATE_ACTIVE_BURST && write_slave_select == 0;
			axi_bus_s0.s_bvalid = axi_bus_m1.s_bvalid && write_slave_select == 0;
			axi_bus_s1.s_awready = axi_bus_m1.s_awready && write_state == STATE_ISSUE_ADDRESS && write_slave_select == 1;
			axi_bus_s1.s_wready = axi_bus_m1.s_wready && write_state == STATE_ACTIVE_BURST && write_slave_select == 1;
			axi_bus_s1.s_bvalid = axi_bus_m1.s_bvalid && write_slave_select == 1;
		end
	end
	
	//
	// Read handling.  Slave interface 1 has priority.
	//
	logic read_selected_slave;  // Which slave interface we are accepting request from
	logic read_selected_master; // Which master interface we are routing to
	logic[7:0] read_burst_length;	// Like axi_arlen, this is number of transfers minus one
	logic[31:0] read_burst_address;
	logic[1:0] read_state;
	wire axi_arready_m = read_selected_master ? axi_bus_m1.s_arready : axi_bus_m0.s_arready;
	wire axi_rready_m = read_selected_master ? axi_bus_m1.m_rready : axi_bus_m0.m_rready;
	wire axi_rvalid_m = read_selected_master ? axi_bus_m1.s_rvalid : axi_bus_m0.s_rvalid;
	
	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			read_state <= STATE_ARBITRATE;

			/*AUTORESET*/
			// Beginning of autoreset for uninitialized flops
			read_burst_address <= 32'h0;
			read_burst_length <= 8'h0;
			read_selected_master <= 1'h0;
			read_selected_slave <= 1'h0;
			// End of automatics
		end
		else if (read_state == STATE_ACTIVE_BURST)
		begin
			// Burst is active.  Check to see when it is finished.
			if (axi_rready_m && axi_rvalid_m)
			begin
				read_burst_length <= read_burst_length - 8'd1;
				if (read_burst_length == 0)
					read_state <= STATE_ARBITRATE;
			end
		end
		else if (read_state == STATE_ISSUE_ADDRESS)
		begin
			// Wait for the slave to accept the address and length
			if (axi_arready_m)
				read_state <= STATE_ACTIVE_BURST;
		end
		else if (axi_bus_s2.m_arvalid)
		begin
			// Start a read burst from slave 2
			read_state <= STATE_ISSUE_ADDRESS;
			read_burst_address <= axi_bus_s2.m_araddr;
			read_burst_length <= axi_bus_s2.m_arlen;
			read_selected_slave <= 2'd2;
			//read_selected_master <= axi_bus_s2.m_araddr[31:28] != 0;
			read_selected_master <= axi_bus_s2.m_araddr >= M1_BASE_ADDRESS;
		end
		else if (axi_bus_s1.m_arvalid)
		begin
			// Start a read burst from slave 1
			read_state <= STATE_ISSUE_ADDRESS;
			read_burst_address <= axi_bus_s1.m_araddr;
			read_burst_length <= axi_bus_s1.m_arlen;
			read_selected_slave <= 2'd1;
			//read_selected_master <= axi_bus_s1.m_araddr[31:28] != 0;
			read_selected_master <= axi_bus_s1.m_araddr >= M1_BASE_ADDRESS;
		end
		else if (axi_bus_s0.m_arvalid)
		begin
			// Start a read burst from slave 0
			read_state <= STATE_ISSUE_ADDRESS;
			read_burst_address <= axi_bus_s0.m_araddr;
			read_burst_length <= axi_bus_s0.m_arlen;
			read_selected_slave <= 2'd0;
			//read_selected_master <= axi_bus_s0.m_araddr[31:28] != 0;
			read_selected_master <= axi_bus_s0.m_araddr >= M1_BASE_ADDRESS;
		end
	end

	always_comb
	begin
		if (read_state == STATE_ARBITRATE)
		begin
			axi_bus_s0.s_rvalid = 0;
			axi_bus_s1.s_rvalid = 0;
			axi_bus_s2.s_rvalid = 0;
			axi_bus_m0.m_rready = 0;
			axi_bus_m1.m_rready = 0;
			axi_bus_s0.s_arready = 0;
			axi_bus_s1.s_arready = 0;
			axi_bus_s2.s_arready = 0;
		end
		else if (read_selected_slave == 0)
		begin
			axi_bus_s0.s_rvalid = axi_rvalid_m;
			axi_bus_s1.s_rvalid = 0;
			axi_bus_s2.s_rvalid = 0;
			axi_bus_m0.m_rready = axi_bus_s0.m_rready && read_selected_master == 0; 
			axi_bus_m1.m_rready = axi_bus_s0.m_rready && read_selected_master == 1;
			axi_bus_s0.s_arready = axi_arready_m && read_state == STATE_ISSUE_ADDRESS;
			axi_bus_s1.s_arready = 0;
			axi_bus_s2.s_arready = 0;
		end
		else if(read_selected_slave == 1)
		begin
			axi_bus_s0.s_rvalid = 0;
			axi_bus_s1.s_rvalid = axi_rvalid_m;
			axi_bus_s2.s_rvalid = 0;
			axi_bus_m0.m_rready = axi_bus_s1.m_rready && read_selected_master == 0; 
			axi_bus_m1.m_rready = axi_bus_s1.m_rready && read_selected_master == 1;
			axi_bus_s0.s_arready = 0;
			axi_bus_s1.s_arready = axi_arready_m && read_state == STATE_ISSUE_ADDRESS;
			axi_bus_s2.s_arready = 0;
		end
		else// if(read_selected_slave == 2)
		begin
			axi_bus_s0.s_rvalid = 0;
			axi_bus_s1.s_rvalid = 0;
			axi_bus_s2.s_rvalid = axi_rvalid_m;
			axi_bus_m0.m_rready = axi_bus_s2.m_rready && read_selected_master == 0; 
			axi_bus_m1.m_rready = axi_bus_s2.m_rready && read_selected_master == 1;
			axi_bus_s0.s_arready = 0;
			axi_bus_s1.s_arready = 0;
			axi_bus_s2.s_arready = axi_arready_m && read_state == STATE_ISSUE_ADDRESS;
		end
	end

	assign axi_bus_m0.m_arvalid = read_state == STATE_ISSUE_ADDRESS && read_selected_master == 0;
	assign axi_bus_m1.m_arvalid = read_state == STATE_ISSUE_ADDRESS && read_selected_master == 1;
	assign axi_bus_m0.m_araddr = read_burst_address;
	assign axi_bus_m1.m_araddr = read_burst_address - M1_BASE_ADDRESS;
	assign axi_bus_s0.s_rdata = read_selected_master ? axi_bus_m1.s_rdata : axi_bus_m0.s_rdata;
	assign axi_bus_s1.s_rdata = read_selected_master ? axi_bus_m1.s_rdata : axi_bus_m0.s_rdata;
	assign axi_bus_s2.s_rdata = read_selected_master ? axi_bus_m1.s_rdata : axi_bus_m0.s_rdata;
	
	always_comb
	begin
		case(read_selected_slave)
			2'd0: 
			begin
				axi_bus_m0.m_arburst = axi_bus_s0.m_arburst;
				axi_bus_m0.m_arsize =  axi_bus_s0.m_arsize;
				axi_bus_m1.m_arburst = axi_bus_s0.m_arburst;
				axi_bus_m1.m_arsize =  axi_bus_s0.m_arsize;
			end
			2'd1: 
			begin
				axi_bus_m0.m_arburst = axi_bus_s1.m_arburst;
				axi_bus_m0.m_arsize =  axi_bus_s1.m_arsize;
				axi_bus_m1.m_arburst = axi_bus_s1.m_arburst;
				axi_bus_m1.m_arsize =  axi_bus_s1.m_arsize;
			end
			2'd2: 
			begin
				axi_bus_m0.m_arburst = axi_bus_s2.m_arburst;
				axi_bus_m0.m_arsize =  axi_bus_s2.m_arsize;
				axi_bus_m1.m_arburst = axi_bus_s2.m_arburst;
				axi_bus_m1.m_arsize =  axi_bus_s2.m_arsize;
			end
		endcase
	end
	//assign axi_bus_m0.m_arburst = read_selected_master ? axi_bus_s1.m_arburst : axi_bus_s0.m_arburst;
	//assign axi_bus_m0.m_arsize = read_selected_master ? axi_bus_s1.m_arsize : axi_bus_s0.m_arsize;
	
	/*
	assign axi_bus_s0.m_arburst = read_selected_master ? axi_bus_m1.m_arburst:axi_bus_m0.m_arburst;
	assign axi_bus_s1.m_arburst = read_selected_master ? axi_bus_m1.m_arburst:axi_bus_m0.m_arburst;
	assign axi_bus_s2.m_arburst = read_selected_master ? axi_bus_m1.m_arburst:axi_bus_m0.m_arburst;
	assign axi_bus_s0.m_arsize = read_selected_master ? axi_bus_m1.m_arsize:axi_bus_m0.m_arsize;
	assign axi_bus_s1.m_arsize = read_selected_master ? axi_bus_m1.m_arsize:axi_bus_m0.m_arsize;
	assign axi_bus_s2.m_arsize = read_selected_master ? axi_bus_m1.m_arsize:axi_bus_m0.m_arsize;
	*/
	// Note that we end up reusing read_burst_length to track how many beats are left
	// later.  At this point, the value of ARLEN should be ignored by slave
	// we are driving, so it won't break anything.
	assign axi_bus_m0.m_arlen = read_burst_length;
	assign axi_bus_m1.m_arlen = read_burst_length;
endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:

