`timescale 1ns / 1ps

`include "aquila_config.vh"

module rap #( parameter BHT_ENTRY_NUM = 256, RAS_ENTRY_NUM = 128, parameter XLEN = 32 )
(
    // System signals
    input               clk_i,
    input               rst_i,
    input               stall_i,
    input               stall_data_hazard_i,

    // from Program_Counter
    input  [XLEN-1 : 0] pc_i, // Addr of the next instruction to be fetched.

    // from Decode
    input               is_jalr_i,
    input               is_jal_i,
    input  [XLEN-1 : 0] dec_pc_i, // Addr of the instr. just processed by decoder.

    // to Program_Counter
    output              jalr_hit_o,
    output [XLEN-1 : 0] jalr_target_addr_o
);

localparam NBITS = $clog2(BHT_ENTRY_NUM);

wire [NBITS-1 : 0]      read_addr;
wire [NBITS-1 : 0]      write_addr;
wire [XLEN-1 : 0]       jalr_inst_tag;
wire                    we_BHT;
wire                    we_RAS;
reg                     BHT_hit_ff, BHT_hit;

// "we" is enabled to add a new entry to the BHT table when
// the decoded branch instruction is not in the BHT.
// CY Hsiang 0220_2020: added "~stall_i" to "we ="
assign we_BHT = ~stall_i & (is_jalr_i) & !BHT_hit;
assign we_RAS = ~stall_i & (is_jal_i);

assign read_addr = pc_i[NBITS+2 : 2];
assign write_addr = dec_pc_i[NBITS+2 : 2];

// ===========================================================================
//  Branch History Table (BHT). Here, we use a direct-mapping cache table to
//  store branch history. Each entry of the table contains two fields:
//  the branch_target_addr and the PC of the branch instruction (as the tag).
//
distri_ram #(.ENTRY_NUM(BHT_ENTRY_NUM), .XLEN(XLEN))
RAS_BHT(
    .clk_i(clk_i),
    .we_i(we_BHT),                  // Write-enabled when the instruction at the Decode
                                //   is a branch and has never been executed before.
    .write_addr_i(write_addr),  // Direct-mapping index for the branch at Decode.
    .read_addr_i(read_addr),    // Direct-mapping Index for the next PC to be fetched.

    .data_i({dec_pc_i}), // Input is not used when 'we' is 0.
    .data_o({jalr_inst_tag})
);

// Delay the BHT hit flag at the Fetch stage for two clock cycles (plus stalls)
// such that it can be reused at the Execute stage for BHT update operation.
always @ (posedge clk_i)
begin
    if (rst_i) begin
        BHT_hit_ff <= 1'b0;
        BHT_hit <= 1'b0;
    end
    else if (!stall_i) begin
        BHT_hit_ff <= jalr_hit_o;
        BHT_hit <= BHT_hit_ff;
    end
end

//return address stack
reg [XLEN-1 : 0] RAS[RAS_ENTRY_NUM - 1 : 0];
reg [RAS_ENTRY_NUM - 1 : 0] sp;
wire [RAS_ENTRY_NUM - 1 : 0]  sp_plus_1 = sp+ 1 == RAS_ENTRY_NUM ? 0 : sp + 1;
wire [RAS_ENTRY_NUM - 1 : 0]  sp_minus_1 = sp - 1 == -1 ? RAS_ENTRY_NUM - 1 : sp - 1;
integer i;
initial begin
    sp = 0;
    for (i = 0; i < RAS_ENTRY_NUM ; i = i + 1)
        RAS[i] <= 0;
end

always @(posedge clk_i)begin
    if(we_RAS)begin
        RAS[sp] <= dec_pc_i + 4;
        sp <= sp_plus_1;
    end
    else if(jalr_hit_o & !stall_i & !stall_data_hazard_i) begin
        sp <= sp_minus_1;
    end
end


// ===========================================================================
//  Outputs signals
//
wire initial_zero = pc_i == 32'b0;
assign jalr_hit_o = (jalr_inst_tag == pc_i) & !initial_zero;
assign jalr_target_addr_o = RAS[sp_minus_1];

endmodule
