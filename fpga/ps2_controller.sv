// 
// Copyright 2015 Jeff Bush
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


//
// PS/2 peripheral.  Only supports receiving
//

module ps2_controller
	#(parameter BASE_ADDRESS = 0)

	(input              clk,
	input               reset,
	
	// IO bus interface
	input [31:0]		io_address,
	input				io_read_en,	
	input [31:0]		io_write_data,
	input				io_write_en,
	output logic[31:0] 	io_read_data,
	
	// PS/2 Interface
	inout               ps2_clk,
	inout               ps2_data);

	localparam STATUS_REG = BASE_ADDRESS;
	localparam DATA_REG = BASE_ADDRESS + 4;
	localparam FIFO_LENGTH = 16;

	typedef enum logic[1:0] {
		STATE_WAIT_START,
		STATE_READ_CHARACTER,
		STATE_READ_PARITY,
		STATE_READ_STOP_BIT
	} receive_state_t;

	logic ps2_clk_sync;
	logic ps2_data_sync;
	logic ps2_clk_prev;
	receive_state_t state_ff;
	logic[2:0] bit_count;
	logic[7:0] receive_byte;
	logic[7:0] dequeue_data;
	logic read_fifo_empty;
	logic fifo_almost_full;
	logic enqueue_en;

	synchronizer #(.WIDTH(2), .RESET_STATE(2'b11)) input_synchronizer(
		.data_i({ps2_clk, ps2_data}),
		.data_o({ps2_clk_sync, ps2_data_sync}),
		.*);

	// If the FIFO hits the almost full threshold, we dequeue an entry (dropping the oldest
	// character). We use the almost full threshold instead of full because the Altera specs
	// say it is not allowed to enqueue into a full FIFO. Although it seems like this should 
	// be legal if read is also asserted, I'm being conservative.
	sync_fifo #(.WIDTH(8), .SIZE(FIFO_LENGTH), .ALMOST_FULL_THRESHOLD(FIFO_LENGTH - 1)) input_fifo(
		.flush_en(0),
		.full(),
		.almost_full(fifo_almost_full),
		.enqueue_en(enqueue_en),
		.value_i(receive_byte),
		.empty(read_fifo_empty),
		.almost_empty(),
		.dequeue_en((io_read_en && io_address == DATA_REG && !read_fifo_empty) || fifo_almost_full),
		.value_o(dequeue_data),
		.*);

	always_comb
	begin
		if (io_address == STATUS_REG)
			io_read_data <= !read_fifo_empty;
		else 
			io_read_data <= dequeue_data;
	end

	always @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			ps2_clk_prev <= 1;
			state_ff <= STATE_WAIT_START;
			bit_count <= 0;
			enqueue_en <= 0;
		end
		else
		begin
			ps2_clk_prev <= ps2_clk_sync;
			enqueue_en <= 0;
			if (ps2_clk_sync == 0 && ps2_clk_prev == 1)
			begin
				// Valid data on the falling edge
				case (state_ff)
					STATE_WAIT_START:
					begin
						if (ps2_data_sync == 0)
						begin
							state_ff <= STATE_READ_CHARACTER;
							bit_count <= 0;
						end
					end

					STATE_READ_CHARACTER:
					begin
						bit_count <= bit_count + 1;
						if (bit_count == 7)
							state_ff <= STATE_READ_PARITY;
							
						receive_byte <= { ps2_data_sync, receive_byte[7:1] };
					end

					STATE_READ_PARITY:
					begin
						// XXX not checking parity
						
						state_ff <= STATE_READ_STOP_BIT;
					end

					STATE_READ_STOP_BIT:
					begin
						// XXX not checking that stop bit is high
					
						state_ff <= STATE_WAIT_START;
						enqueue_en <= 1;
					end
				endcase
			end
		end
	end
endmodule

// Local Variables:
// verilog-library-flags:("-y ../core" "-y ../testbench")
// End:

