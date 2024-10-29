`timescale 1ns / 1ps
// =============================================================================
//  Program : execute.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Execution Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/29/2019, by Chun-Jen Tsai:
//    Merges the pipeline register module 'execute_memory' into the 'execute'
//    module.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

module execute #( parameter XLEN = 32 )
(
    //  Processor clock and reset signals.
    input                   clk_i,
    input                   rst_i,

    // Pipeline stall signal.
    input                   stall_i,

    // Pipeline flush signal.
    input                   flush_i,

    // From Decode.
    input  [XLEN-1 : 0]     imm_i,
    input  [ 1 : 0]         inputA_sel_i,
    input  [ 1 : 0]         inputB_sel_i,
    input  [ 2 : 0]         operation_sel_i,
    input                   alu_muldiv_sel_i,
    input                   shift_sel_i,
    input                   is_branch_i,
    input                   is_jal_i,
    input                   is_jalr_i,
    input                   is_ret_i,
    input                   branch_hit_i,
    input                   ret_hit_i,
    input                   branch_decision_i,

    input                   regfile_we_i,
    input  [ 2 : 0]         regfile_input_sel_i,
    input                   we_i,
    input                   re_i,
    input  [ 1 : 0]         dsize_sel_i,
    input                   signex_sel_i,
    input  [ 4 : 0]         rd_addr_i,
    input                   is_fencei_i,
    input                   is_amo_i,
    input  [4 : 0]          amo_type_i,


    // From CSR.
    input  [ 4 : 0]         csr_imm_i,
    input                   csr_we_i,
    input  [11 : 0]         csr_we_addr_i,

    // From the Forwarding Unit.
    input  [XLEN-1 : 0]     rs1_data_i,
    input  [XLEN-1 : 0]     rs2_data_i,
    input  [XLEN-1 : 0]     csr_data_i,

    // To the Program Counter Unit.
    output [XLEN-1 : 0]     branch_restore_pc_o,    // to PC only
    output [XLEN-1 : 0]     branch_target_addr_o,   // to PC and BPU

    // To the Pipeline Control and the Branch Prediction units.
    output                  is_branch_o,
    output                  branch_taken_o,
    output                  branch_misprediction_o,

    // rap
    input     [XLEN-1 : 0]  dec_cur_pc_i,
    output                  ret_misprediction_o,
    output                  ret_executed,

    // Pipeline stall signal generator, activated when executing
    //    multicycle mul, div and rem instructions.
    output                  stall_from_exe_o,

    // Signals to D-memory.
    output reg              we_o,
    output reg              re_o,
    output reg              is_fencei_o,
    output reg              is_amo_o,
    output reg [ 4 : 0]     amo_type_o,

    // Signals to Memory Alignment unit.
    output reg [XLEN-1 : 0] rs2_data_o,
    output reg [XLEN-1 : 0] addr_o,
    output reg [ 1 : 0]     dsize_sel_o,
    
    // Signals to Memory Writeback Pipeline.
    output reg [ 2 : 0]     regfile_input_sel_o,
    output reg              regfile_we_o,
    output reg [ 4 : 0]     rd_addr_o,
    output reg [XLEN-1 : 0] p_data_o,

    output reg              csr_we_o,
    output reg [11 : 0]     csr_we_addr_o,
    output reg [XLEN-1 : 0] csr_we_data_o,

    // to Memory_Write_Back_Pipeline
    output reg              signex_sel_o,

    // PC of the current instruction.
    input  [XLEN-1 : 0]     pc_i,
    output reg [XLEN-1 : 0] pc_o,

    // System Jump operation
    input                   sys_jump_i,
    input  [ 1 : 0]         sys_jump_csr_addr_i,
    output reg              sys_jump_o,
    output reg [ 1 : 0]     sys_jump_csr_addr_o,

    // Has instruction fetch being successiful?
    input                   fetch_valid_i,
    output reg              fetch_valid_o,

    // Exception info passed from Decode to Memory.
    input                   xcpt_valid_i,
    input  [ 3 : 0]         xcpt_cause_i,
    input  [XLEN-1 : 0]     xcpt_tval_i,
    output reg              xcpt_valid_o,
    output reg [ 3 : 0]     xcpt_cause_o,
    output reg [XLEN-1 : 0] xcpt_tval_o
);

reg  [XLEN-1 : 0] inputA, inputB;
wire [XLEN-1 : 0] alu_result;
wire [XLEN-1 : 0] muldiv_result;
wire              compare_result, stall_from_muldiv, muldiv_ready;

wire [XLEN-1 : 0] result;
wire [XLEN-1 : 0] mem_addr;

always @(*)
begin
    case (inputA_sel_i)
        3'd0: inputA = 0;
        3'd1: inputA = pc_i;
        3'd2: inputA = rs1_data_i;
        default: inputA = 0;
    endcase
end

always @(*)
begin
    case (inputB_sel_i)
        3'd0: inputB = imm_i;
        3'd1: inputB = rs2_data_i;
        3'd2: inputB = ~rs2_data_i + 1'b1;
        default: inputB = 0;
    endcase
end

// branch target address generate by alu adder
wire [2: 0] alu_operation = (is_branch_i | is_jal_i | is_jalr_i)? 3'b000 : operation_sel_i;
wire [2: 0] muldiv_operation = operation_sel_i;
wire muldiv_req = alu_muldiv_sel_i & !muldiv_ready;
wire [2: 0] branch_operation = operation_sel_i;

// ===============================================================================
//  ALU Regular operation
//
alu ALU(
    .a_i(inputA),
    .b_i(inputB),
    .operation_sel_i(alu_operation),
    .shift_sel_i(shift_sel_i),
    .alu_result_o(alu_result)
);

// ===============================================================================
//   MulDiv
//
muldiv MulDiv(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .stall_i(stall_i),
    .a_i(inputA),
    .b_i(inputB),
    .req_i(muldiv_req),
    .operation_sel_i(muldiv_operation),
    .muldiv_result_o(muldiv_result),
    .ready_o(muldiv_ready)
);

// ==============================================================================
//  BCU
//
bcu BCU(
    .a_i(rs1_data_i),
    .b_i(rs2_data_i),
    .operation_sel_i(branch_operation),
    .compare_result_o(compare_result)
);

// ===============================================================================
//  AGU & Output signals
//
assign mem_addr = rs1_data_i + imm_i;     // The target addr of memory load/store
assign branch_target_addr_o = alu_result; // The target addr of BRANCH, JAL, JALR
assign branch_restore_pc_o = pc_i + 'd4;  // The next PC of instruction, and the
                                          // restore PC if mispredicted branch taken.

assign is_branch_o = is_branch_i | is_jal_i;
assign branch_taken_o = (is_branch_i & compare_result) | is_jal_i | is_jalr_i;
assign branch_misprediction_o = branch_hit_i & (branch_decision_i ^ branch_taken_o);

assign result = alu_muldiv_sel_i ? muldiv_result : alu_result;
assign stall_from_exe_o = alu_muldiv_sel_i & !muldiv_ready;

// ===============================================================================
//  RAP signal
//
assign ret_misprediction_o = ret_hit_i & (branch_target_addr_o != dec_cur_pc_i) & is_ret_i;
assign ret_executed = ret_hit_i  & is_ret_i;
// ===============================================================================
//  CSR
//
wire [XLEN-1 : 0] csr_inputA = csr_data_i;
wire [XLEN-1 : 0] csr_inputB = operation_sel_i[2] ? {27'b0, csr_imm_i} : rs1_data_i;
reg  [XLEN-1 : 0] csr_update_data;

always @(*)
begin
    case (operation_sel_i[1: 0])
        `CSR_RW:
            csr_update_data = csr_inputB;
        `CSR_RS:
            csr_update_data = csr_inputA | csr_inputB;
        `CSR_RC:
            csr_update_data = csr_inputA & ~csr_inputB;
        default:
            csr_update_data = csr_inputA;
    endcase
end

// ===============================================================================
//  Output registers to the Memory stage
//
always @(posedge clk_i)
begin
    if (rst_i || (flush_i && !stall_i)) // stall has higher priority than flush.
    begin
        we_o <= 0;
        re_o <= 0;
        rs2_data_o <= 0;
        addr_o <= 0;
        dsize_sel_o <= 0;
        signex_sel_o <= 0;
        regfile_input_sel_o <= 0;
        regfile_we_o <= 0;
        rd_addr_o <= 0;
        is_fencei_o <= 0;
        is_amo_o <= 0;
        amo_type_o <= 0;

        sys_jump_o <= 0;
        sys_jump_csr_addr_o <= 0;
        xcpt_valid_o <= 0;
        xcpt_cause_o <= 0;
        xcpt_tval_o <= 0;
        pc_o <= 0;
        fetch_valid_o <= 0;
        csr_we_o <= 0;
        csr_we_addr_o <= 0;
        csr_we_data_o <= 0;
    end
    else if (stall_i || stall_from_exe_o)
    begin
        we_o <= we_o;
        re_o <= re_o;
        rs2_data_o <= rs2_data_o;
        addr_o <= addr_o;
        dsize_sel_o <= dsize_sel_o;
        signex_sel_o <= signex_sel_o;
        regfile_input_sel_o <= regfile_input_sel_o;
        regfile_we_o <= regfile_we_o;
        rd_addr_o <= rd_addr_o;
        is_fencei_o <= is_fencei_o;
        is_amo_o <= is_amo_o;
        amo_type_o <= amo_type_o;

        sys_jump_o <= sys_jump_o;
        sys_jump_csr_addr_o <= sys_jump_csr_addr_o;
        xcpt_valid_o <= xcpt_valid_o;
        xcpt_cause_o <= xcpt_cause_o;
        xcpt_tval_o <= xcpt_tval_o;
        pc_o <= pc_o;
        fetch_valid_o <= fetch_valid_o;
        csr_we_o <= csr_we_o;
        csr_we_addr_o <= csr_we_addr_o;
        csr_we_data_o <= csr_we_data_o;
    end
    else
    begin
        we_o <= we_i ;
        re_o <= re_i;
        rs2_data_o <= rs2_data_i;
        addr_o  <= mem_addr;
        dsize_sel_o <= dsize_sel_i;
        signex_sel_o <= signex_sel_i;
        regfile_input_sel_o <= regfile_input_sel_i;
        regfile_we_o <= regfile_we_i;
        rd_addr_o <= rd_addr_i;
        is_fencei_o <= is_fencei_i;
        is_amo_o <= is_amo_i;
        amo_type_o <= amo_type_i;

        sys_jump_o <= sys_jump_i;
        sys_jump_csr_addr_o <= sys_jump_csr_addr_i;
        xcpt_valid_o <= xcpt_valid_i;
        xcpt_cause_o <= xcpt_cause_i;
        xcpt_tval_o <= xcpt_tval_i;
        pc_o <= pc_i;
        fetch_valid_o <= fetch_valid_i;
        csr_we_o <= csr_we_i;
        csr_we_addr_o <= csr_we_addr_i;
        csr_we_data_o <= csr_update_data;
    end
end

always @(posedge clk_i)
begin
    if (rst_i || (flush_i && !stall_i)) // stall has higher priority than flush.
    begin
        p_data_o <= 0;  // data from processor
    end
    else if (stall_i || stall_from_exe_o)
    begin
        p_data_o <= p_data_o;
    end
    else
    begin
        case (regfile_input_sel_i)
            3'b011: p_data_o <= branch_restore_pc_o;
            3'b100: p_data_o <= result;
            3'b101: p_data_o <= csr_data_i;
            default: p_data_o <= 0;
        endcase
    end
end


//modified code for lab2
// pc = pc_i
reg start_cnt_area;
reg end_cnt_area;
always @(posedge clk_i) begin
    if (rst_i) begin
        start_cnt_area <= 0;
        end_cnt_area <= 0;
    end else begin
        //0000_1088 main
       if (pc_i == 32'h0000_1088) begin
//        if (pc_i == 32'h0000_121c) begin
//         if (pc_i == 32'h0000_1044) begin

            start_cnt_area <= 1;
        end
        // 0000_01e8 or 0000_01ec end 
       if (pc_i == 32'h0000_1798) begin
//            if (pc_i == 32'h0000_1900) begin
//         if (pc_i == 32'h0000_1074) begin

               end_cnt_area <= 1;
        end
    end
end
wire total_cycle_flag;
//wire core_list_find_flag;
//wire core_list_reverse_flag;
//wire core_state_transition_flag;
//wire matrix_mul_matrix_bitextract_flag;
//wire crcu8_flag;

assign total_cycle_flag = (start_cnt_area && (!end_cnt_area));
//// assign core_list_find_flag = (pc_i >= 32'h0000_1cfc) && (pc_i <= 32'h0000_1d4c) && total_cycle_flag ;
//assign core_list_find_flag = (pc_i == 32'h0000_1cfc) && total_cycle_flag ;
//// assign core_list_reverse_flag = (pc_i >= 32'h0000_1d50) && (pc_i <= 32'h0000_1d70) && total_cycle_flag;
//assign core_list_reverse_flag = (pc_i == 32'h0000_1d50) && total_cycle_flag;
//assign core_state_transition_flag = (pc_i >= 32'h0000_29f4) && (pc_i <= 32'h0000_2cdc) && total_cycle_flag;
//assign matrix_mul_matrix_bitextract_flag = (pc_i >= 32'h0000_2650) && (pc_i <= 32'h0000_270c) && total_cycle_flag;
//assign crcu8_flag = (pc_i >= 32'h0000_19b4) && (pc_i <= 32'h0000_19f8) && total_cycle_flag;
wire ret_flag = total_cycle_flag && is_ret_i;
wire ret_miss = ret_flag && (!ret_hit_i);
wire ret_mispredict = ret_flag && (ret_misprediction_o);
wire branch =  total_cycle_flag & (is_branch_i | is_jal_i);
//// //cycle count
//(* mark_debug = "true" *) reg [32-1:0] cl_find_cnt;
//(* mark_debug = "true" *) reg [32-1:0] cl_reverse_cnt;
//(* mark_debug = "true" *) reg [32-1:0] cs_transition_cnt;
//(* mark_debug = "true" *) reg [32-1:0] matrix_cnt;
//(* mark_debug = "true" *) reg [32-1:0] crcu8_cnt;
//always @(posedge clk_i) begin
//    if (rst_i) begin
//        cl_find_cnt <= 0;
//        cl_reverse_cnt <= 0;
//        cs_transition_cnt <= 0;
//        matrix_cnt <= 0;
//        crcu8_cnt <= 0;
//    end else begin
//        if (core_list_find_flag) begin
//            cl_find_cnt <= cl_find_cnt + 1;
//        end
//        if (core_list_reverse_flag) begin
//            cl_reverse_cnt <= cl_reverse_cnt + 1;
//        end
//        if (core_state_transition_flag) begin
//            cs_transition_cnt <= cs_transition_cnt + 1;
//        end
//        if (matrix_mul_matrix_bitextract_flag) begin
//            matrix_cnt <= matrix_cnt + 1;
//        end
//        if (crcu8_flag) begin
//            crcu8_cnt <= crcu8_cnt + 1;
//        end
//    end
//end

(* mark_debug = "true" *) reg [64-1:0] total_cycle_cnt;
(* mark_debug = "true" *) reg [32-1:0] stall_cycle_cnt;

(* mark_debug = "true" *) reg [16-1:0] ret_miss_cnt;
(* mark_debug = "true" *) reg [16-1:0] ret_mispredict_cnt;
(* mark_debug = "true" *) reg [32-1:0] ret_cycle_cnt;
(* mark_debug = "true" *) reg [32-1:0] branch_cycle_cnt;
always @(posedge clk_i)begin
    if(rst_i)begin
        total_cycle_cnt <= 0;
        stall_cycle_cnt <= 0;
        ret_miss_cnt <= 0;
        ret_mispredict_cnt <= 0;
        ret_cycle_cnt <= 0;
        branch_cycle_cnt <= 0;
    end
    if(total_cycle_flag)begin
        total_cycle_cnt <= total_cycle_cnt + 1;
        if(stall_i) stall_cycle_cnt <= stall_cycle_cnt + 1;
    end
    if(ret_flag) ret_cycle_cnt <= ret_cycle_cnt + 1;
    if(ret_miss) ret_miss_cnt <= ret_miss_cnt + 1;
    if(ret_mispredict) ret_mispredict_cnt <= ret_mispredict_cnt + 1;
    if(branch) branch_cycle_cnt <= branch_cycle_cnt + 1;
end

// wire forward_jump_flag = total_cycle_flag && is_branch_i && (pc_i <  branch_target_addr_o);
// wire backward_jump_flag = total_cycle_flag && is_branch_i && (pc_i >  branch_target_addr_o);
// wire jump_flag = total_cycle_flag && (is_jal_i);
// wire forward_jump_mispredict = forward_jump_flag && (branch_misprediction_o);
// wire forward_jump_miss = forward_jump_flag && (!branch_hit_i);
// wire backward_jump_mispredict = backward_jump_flag && (branch_misprediction_o);
// wire backward_jump_miss = backward_jump_flag && (!branch_hit_i);
// wire jump_mispredict = jump_flag && (branch_misprediction_o);
// wire jump_miss = jump_flag && (!branch_hit_i);
// (* mark_debug = "true" *) reg [32-1:0] forward_jump_cnt;
// (* mark_debug = "true" *) reg [32-1:0] backward_jump_cnt;
// (* mark_debug = "true" *) reg [32-1:0] jump_cnt;

// (* mark_debug = "true" *) reg [32-1:0] forward_jump_miss_cnt;
// (* mark_debug = "true" *) reg [32-1:0] backward_jump_miss_cnt;
// (* mark_debug = "true" *) reg [32-1:0] jump_miss_cnt;

// (* mark_debug = "true" *) reg [32-1:0] forward_jump_mispredict_cnt;
// (* mark_debug = "true" *) reg [32-1:0] backward_jump_mispredict_cnt;
// (* mark_debug = "true" *) reg [32-1:0] jump_mispredict_cnt;

// always @(posedge clk_i) begin
//     if (rst_i) begin
//         forward_jump_cnt <= 0;
//         backward_jump_cnt <= 0;
//         jump_cnt <= 0;
//     end else begin
//         if(total_cycle_flag ) begin
//             total_cycle_cnt <= total_cycle_cnt + 1;
//             if(stall_i) stall_cycle_cnt <= stall_cycle_cnt + 1;
//         end 
//         if (forward_jump_flag) begin
//             forward_jump_cnt <= forward_jump_cnt + 1;
//         end
//         if (backward_jump_flag) begin
//             backward_jump_cnt <= backward_jump_cnt + 1;
//         end
//         if (jump_flag) begin
//             jump_cnt <= jump_cnt + 1;
//         end
//     end
// end

// always @(posedge clk_i) begin
//     if (rst_i) begin
//         forward_jump_miss_cnt <= 0;
//         backward_jump_miss_cnt <= 0;
//         jump_miss_cnt <= 0;
//     end else begin
//         if (forward_jump_miss) begin
//             forward_jump_miss_cnt <= forward_jump_miss_cnt + 1;
//         end
//         if (backward_jump_miss) begin
//             backward_jump_miss_cnt <= backward_jump_miss_cnt + 1;
//         end
//         if (jump_miss) begin
//             jump_miss_cnt <= jump_miss_cnt + 1;
//         end
//     end
// end

// always @(posedge clk_i) begin
//     if (rst_i) begin
//         forward_jump_mispredict_cnt <= 0;
//         backward_jump_mispredict_cnt <= 0;
//         jump_mispredict_cnt <= 0;
//     end else begin
//         if (forward_jump_mispredict) begin
//             forward_jump_mispredict_cnt <= forward_jump_mispredict_cnt + 1;
//         end
//         if (backward_jump_mispredict) begin
//             backward_jump_mispredict_cnt <= backward_jump_mispredict_cnt + 1;
//         end
//         if (jump_mispredict) begin
//             jump_mispredict_cnt <= jump_mispredict_cnt + 1;
//         end
//     end
// end


/*
wire operation_beq = (operation_sel_i == 3'b000);
wire operation_bne = (operation_sel_i == 3'b001);
wire operation_blt = (operation_sel_i == 3'b100);
wire operation_bge = (operation_sel_i == 3'b101);
wire operation_bltu = (operation_sel_i == 3'b110);
wire operation_bgeu = (operation_sel_i == 3'b111);
*/
//wire beq_flag = branch_operation == 3'b000 && total_cycle_flag && is_branch_i;
//wire bne_flag = branch_operation == 3'b001&& total_cycle_flag&& is_branch_i;
//wire blt_flag = branch_operation == 3'b100&& total_cycle_flag&& is_branch_i;
//wire bge_flag = branch_operation == 3'b101&& total_cycle_flag&& is_branch_i;
//wire bltu_flag = branch_operation == 3'b110&& total_cycle_flag&& is_branch_i;
//wire bgeu_flag = branch_operation == 3'b111&& total_cycle_flag&& is_branch_i;

//(* mark_debug = "true" *) reg [32-1:0] beq_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bne_cnt;
//(* mark_debug = "true" *) reg [32-1:0] blt_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bge_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bltu_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bgeu_cnt;

//(* mark_debug = "true" *) reg [32-1:0] beq_miss_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bne_miss_cnt;
//(* mark_debug = "true" *) reg [32-1:0] blt_miss_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bge_miss_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bltu_miss_cnt;
//(* mark_debug = "true" *) reg [32-1:0] bgeu_miss_cnt;

//always @(posedge clk_i) begin
//    if (rst_i) begin
//        beq_cnt <= 0;
//        bne_cnt <= 0;
//        blt_cnt <= 0;
//        bge_cnt <= 0;
//        bltu_cnt <= 0;
//        bgeu_cnt <= 0;
//        beq_miss_cnt <= 0;
//        bne_miss_cnt <= 0;
//        blt_miss_cnt <= 0;
//        bge_miss_cnt <= 0;
//        bltu_miss_cnt <= 0;
//        bgeu_miss_cnt <= 0;
//    end else begin
//        if(beq_flag) beq_cnt <= beq_cnt + 1;
//        if(beq_flag && (branch_misprediction_o || !branch_hit_i)) beq_miss_cnt <= beq_miss_cnt + 1;
        
//        if(bne_flag) bne_cnt <= bne_cnt + 1;
//        if(bne_flag && (branch_misprediction_o || !branch_hit_i)) bne_miss_cnt <= bne_miss_cnt + 1;

//        if(blt_flag) blt_cnt <= blt_cnt + 1;
//        if(blt_flag && (branch_misprediction_o || !branch_hit_i)) blt_miss_cnt <= blt_miss_cnt + 1;

//        if(bge_flag) bge_cnt <= bge_cnt + 1;
//        if(bge_flag && (branch_misprediction_o || !branch_hit_i)) bge_miss_cnt <= bge_miss_cnt + 1;

//        if(bltu_flag) bltu_cnt <= bltu_cnt + 1;
//        if(bltu_flag && (branch_misprediction_o || !branch_hit_i)) bltu_miss_cnt <= bltu_miss_cnt + 1;

//        if(bgeu_flag) bgeu_cnt <= bgeu_cnt + 1;
//        if(bgeu_flag && (branch_misprediction_o || !branch_hit_i)) bgeu_miss_cnt <= bgeu_miss_cnt + 1;
//    end
//end



endmodule
