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
// Maintains a least recently used list for each cache set. Used to determine
// which cache way to load new cache lines into.
//
// There are two interfaces that update the LRU. The old contents of the LRU for
// the updated set must be fetched a cycle before updating.
//
// Fill:
// When a cache line is to be filled, fill_en/fill_set are asserted.
// One cycle later, this module asserts fill_way to indicate the least
// recently used way (which should be replaced). It will automatically move
// that way to the MRU.
//
// Access: 
// During the first cycle of a cache loads, the client asserts
// access_en/access_set. One cycle later, if there was a cache hit, it asserts
// update_en/update_way to update the accessed way to the MRU position. It is 
// illegal to assert update_en if access_en was not asserted in the previous
// cycle.
//
// If both fill_en and access_en are asserted simultaneously, fill wins. This is
// important to avoid evicting newly loaded lines when there are many fills back
// to back and to avoid livelock.
//

module cache_lru
	#(parameter NUM_SETS = 1,
	parameter NUM_WAYS = 4,	// Must be 1, 2, 4, or 8
	parameter SET_INDEX_WIDTH = $clog2(NUM_SETS),
	parameter WAY_INDEX_WIDTH = $clog2(NUM_WAYS))
	(input                                clk,
	input                                 reset,
	
	// Fill interface
	input                                 fill_en,
	input [SET_INDEX_WIDTH - 1:0]         fill_set,
	output logic [WAY_INDEX_WIDTH - 1:0]  fill_way,
	
	// Access interface
	input                                 access_en,
	input [SET_INDEX_WIDTH - 1:0]         access_set,
	input                                 access_update_en,
	input [WAY_INDEX_WIDTH - 1:0]         access_update_way);

	localparam LRU_FLAG_BITS = 
		NUM_WAYS == 1 ? 1 :
		NUM_WAYS == 2 ? 1 :
		NUM_WAYS == 4 ? 3 :
		7;	// NUM_WAYS = 8
	
	logic[LRU_FLAG_BITS - 1:0] lru_flags;
	logic update_lru_en;
	logic [SET_INDEX_WIDTH - 1:0] update_set;
	logic[LRU_FLAG_BITS - 1:0] update_flags;
	logic [SET_INDEX_WIDTH - 1:0] read_set;
	logic read_en;
	logic was_fill;
	logic was_access;
	logic[WAY_INDEX_WIDTH - 1:0] new_mru;
	
	assign read_en = access_en || fill_en;
	assign read_set = fill_en ? fill_set : access_set;
	assign new_mru = was_fill ? fill_way : access_update_way;
	assign update_lru_en = was_fill || access_update_en;

	// This uses a pseudo-LRU algorithm
	// The current state of each set is represented by 3 bits.  Imagine a tree:
	//
	//        b
	//      /   \
	//     a     c
	//    / \   / \
	//   0   1 2   3
	//
	// The letters a, b, and c represent the 3 bits which indicate a path to 
	// the *least recently used* element. A 0 stored in a node indicates the 
	// left node and a 1 the right. Each time an element is moved to the MRU, 
	// the bits along its path are set to the opposite direction.
	//
	sram_1r1w #(
		.DATA_WIDTH(LRU_FLAG_BITS), 
		.SIZE(NUM_SETS), 
		.READ_DURING_WRITE("NEW_DATA")
	) lru_data(
		// Fetch existing flags
		.read_en(read_en),
		.read_addr(read_set),
		.read_data(lru_flags),

		// Update LRU (from next stage)
		.write_en(update_lru_en),
		.write_addr(update_set),
		.write_data(update_flags),
		.*);
	
	generate
		if (NUM_WAYS == 1)
		begin
			assign fill_way = 0;
			assign update_flags = 0;
		end
		else if (NUM_WAYS == 2)
		begin
			assign fill_way = !lru_flags[0];
			assign update_flags[0] = !new_mru;
		end
		else if (NUM_WAYS == 4)
		begin
			always_comb
			begin
				casez (lru_flags)
					3'b00?: fill_way = 0;
					3'b10?: fill_way = 1;
					3'b?10: fill_way = 2;
					3'b?11: fill_way = 3;
					default: fill_way = '0;
				endcase
			end

			always_comb
			begin
				case (new_mru)
					2'd0: update_flags = { 2'b11, lru_flags[0] };
					2'd1: update_flags = { 2'b01, lru_flags[0] };
					2'd2: update_flags = { lru_flags[2], 2'b01 };
					2'd3: update_flags = { lru_flags[2], 2'b00 };
					default: update_flags = '0;
				endcase
			end
		end
		else if (NUM_WAYS == 8)
		begin
			always_comb
			begin
				casez (lru_flags)
					7'b00?0???: fill_way = 0;
					7'b10?0???: fill_way = 1;
					7'b?100???: fill_way = 2;
					7'b?110???: fill_way = 3;
					7'b???100?: fill_way = 4;
					7'b???110?: fill_way = 5;
					7'b???1?10: fill_way = 6;
					7'b???1?11: fill_way = 7;
					default: fill_way = '0;
				endcase
			end

			always_comb
			begin
				case (new_mru)
					3'd0: update_flags = { 2'b11, lru_flags[5], 1'b1, lru_flags[2:0] };
					3'd1: update_flags = { 2'b01, lru_flags[5], 1'b1, lru_flags[2:0] };
					3'd2: update_flags = { lru_flags[6], 3'b011, lru_flags[2:0] };
					3'd3: update_flags = { lru_flags[6], 3'b001, lru_flags[2:0] };
					3'd4: update_flags = { lru_flags[6:4], 3'b011, lru_flags[0] };
					3'd5: update_flags = { lru_flags[6:4], 3'b010, lru_flags[0] };
					3'd6: update_flags = { lru_flags[6:4], 2'b00, lru_flags[1], 1'b1 }; 
					3'd7: update_flags = { lru_flags[6:4], 2'b00, lru_flags[1], 1'b0 };
					default: update_flags = '0;
				endcase
			end
		end
		// XXX does not flag error on invalid number of ways
	endgenerate

	always_ff @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			update_set <= 0;
			was_fill <= 0;
			was_access <= 0;
		end
		else
		begin
			// Verify we don't attempt to update when the last cycle didn't 
			// perform an access.
			assert(!(access_update_en && !was_access));
			was_access <= access_en;	// Debug only

			update_set <= read_set;
			was_fill <= fill_en;
		end
	end
endmodule

// Local Variables:
// verilog-typedef-regexp:"_t$"
// End:
