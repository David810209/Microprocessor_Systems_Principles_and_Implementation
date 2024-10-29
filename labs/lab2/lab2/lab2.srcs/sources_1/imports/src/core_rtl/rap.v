`timescale 1ns / 1ps

`include "aquila_config.vh"

module rap #( parameter BHT_ENTRY_NUM = 256, RAS_ENTRY_NUM = 4, parameter XLEN = 32 )
(
    // System signals
    input               clk_i,
    input               rst_i,
    input               stall_i,
    input               stall_data_hazard_i,

    // from Program_Counter
    input  [XLEN-1 : 0] pc_i, // Addr of the next instruction to be fetched.

    // from Decode
    input               is_ret_i,
    input               is_jal_i,
    input  [XLEN-1 : 0] dec_pc_i, // Addr of the instr. just processed by decoder.

    //from execution
    input               exe_ret_executed_i,

    // to Program_Counter
    output              ret_hit_o,
    output [XLEN-1 : 0] ret_target_addr_o,

    //from plc
    input               flush_i
);

localparam NBITS = $clog2(BHT_ENTRY_NUM);

wire [NBITS-1 : 0]      read_addr;
wire [NBITS-1 : 0]      write_addr;
wire [XLEN-1 : 0]       ret_inst_tag;
wire                    we_BHT;
wire                    we_RAS;
reg                     BHT_hit_ff, BHT_hit;

// "we" is enabled to add a new entry to the BHT table when
// the decoded branch instruction is not in the BHT.
// CY Hsiang 0220_2020: added "~stall_i" to "we ="
assign we_BHT = ~stall_i & (is_ret_i) & !BHT_hit;
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
    .data_o({ret_inst_tag})
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
        BHT_hit_ff <= ret_hit_o;
        BHT_hit <= BHT_hit_ff;
    end
end

//return address stack
wire recover = flush_i & sp2 != 0 & !stall_i; //branch flush and backup storage is not empty

reg [XLEN-1 : 0] RAS[RAS_ENTRY_NUM - 1 : 0];
reg [RAS_ENTRY_NUM - 1 : 0] sp;
wire [RAS_ENTRY_NUM - 1 : 0]  sp_plus_1 = sp + 1 == RAS_ENTRY_NUM ? 0 : sp + 1;
wire [RAS_ENTRY_NUM - 1 : 0]  sp_minus_1 = sp  == 0 ? RAS_ENTRY_NUM - 1 : sp - 1;
integer i;
initial begin
    sp = 0;
    for (i = 0; i < RAS_ENTRY_NUM ; i = i + 1)
        RAS[i] <= 0;
end

always @(posedge clk_i)begin
    //when jal(function call) is taken push return address to ras
    if(we_RAS)begin
        RAS[sp] <= dec_pc_i + 4;
        sp <= sp_plus_1;
    end
`ifdef ENABLE_BS
    // if branch flush and backup storage is not empty
   else if(recover)begin
       RAS[sp] <= bs_top; //recover from the top of bs
       sp <= sp_plus_1;
   end
`endif 
    //ret instruction hit at fetch stage then pop stack top and run it
    else if(ret_hit_o & !stall_i & !stall_data_hazard_i) begin
        sp <= sp_minus_1;
    end
end

//backup storage
reg [XLEN-1 : 0] BS[4- 1 : 0];
reg [4- 1 : 0] sp2;
wire [4 - 1 : 0]  sp2_plus_1 = sp2 == 3 ? 0 : sp2 + 1;
wire [4 - 1 : 0]  sp2_minus_1 = sp2 == 0 ? 3 : sp2 - 1;
integer j;
initial begin
    sp2 = 0;
    for (j = 0; j < 4; j = j+ 1)
        BS[j] <= 0;
end

always @(posedge clk_i)begin
    //ret instruction hit at fetch stage then push ras top in
    if(ret_hit_o & !stall_i & !stall_data_hazard_i)begin
        BS[sp2] <= ret_target_addr_o;
        sp2 <= sp2_plus_1;
    end
    //1: exe_ret is executed(if it actually run and not flushed) then pop the backup storage to clear
    //2: if recover then pop bs top and push it back to ras
    else if((exe_ret_executed_i & !stall_i) | recover) begin
        sp2 <= sp2_minus_1;
    end
end
wire [XLEN-1 : 0] bs_top = BS[sp2_minus_1];

// ===========================================================================
//  Outputs signals
//
wire initial_zero = pc_i == 32'b0;
assign ret_hit_o = (ret_inst_tag == pc_i) & !initial_zero;
assign ret_target_addr_o = RAS[sp_minus_1];

endmodule
