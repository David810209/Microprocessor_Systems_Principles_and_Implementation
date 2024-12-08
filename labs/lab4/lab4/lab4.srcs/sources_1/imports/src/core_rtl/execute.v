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
    input                   branch_hit_i,
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
    output reg [XLEN-1 : 0] xcpt_tval_o,

    output total_flag_o,

    input [XLEN-1 : 0]      test_pc_i
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
//--------------------------------------------------------------
// profiler part of hw4
//--------------------------------------------------------------

//total context switch overhead
`define USING_SEMAPHORE
reg context_switch_flag;

reg total_flag;
assign total_flag_o = total_flag;

(* mark_debug = "true" *) reg [32-1:0] context_switch_cycle;

(* mark_debug = "true" *) reg [32-1:0] context_switch_cnt;
(* mark_debug = "true" *) reg [64-1:0] total_cycle;
reg t1_done, t2_done;
wire finish = t1_done && t2_done;
// wire finish = done;
(* mark_debug = "true" *) reg  done;
always @(posedge clk_i) begin
    if (rst_i) begin
        context_switch_flag <= 0;
        context_switch_cycle <= 0;
        context_switch_cnt <= 0;
        total_cycle<= 0;
        total_flag <= 0;
        t1_done <= 0;
        t2_done <= 0;
        done <= 0;
    end
    else begin
        // 80001448 <main>:
        // 800013f4 <main>:

        //rtos_run2
        //800016fc <main>:
        `ifdef USING_SEMAPHORE
        if(test_pc_i == 32'h8000_1448)
        `else
        if(test_pc_i == 32'h8000_13f4)
        `endif

        // if(pc_i == 32'h8000_16fc)
         begin
            total_flag <= 1;
        end
        //both task done
        if(finish) begin
            total_flag <= 0;
        end
        if(total_flag) begin
            total_cycle <= total_cycle + 1;
        end

        if (context_switch_flag) begin
            context_switch_cycle <= context_switch_cycle + 1;
        end
        //task1- 80001314:	4550306f          	j	80004f68 <vTaskDelete>
        //800012a0:	4750306f          	j	80004f14 <vTaskDelete>
        `ifdef USING_SEMAPHORE
        if(test_pc_i == 32'h8000_1314 && !t1_done) t1_done <= 1; 
        `else
        if(test_pc_i == 32'h8000_12a0 && !t1_done) t1_done <= 1;
        `endif

        //task2- 800010e4:	  6850306f          	j	80004f68 <vTaskDelete>
        //800010c0:	6550306f          	j	80004f14 <vTaskDelete>
        `ifdef USING_SEMAPHORE
        if(test_pc_i == 32'h8000_10e4 && !t2_done) t2_done <= 1; 
        `else
        if(test_pc_i == 32'h8000_10c0 && !t2_done) t2_done <= 1;
        `endif

        // 80007200 <freertos_risc_v_trap_handler>

        //rtos_run2
        //80007600 <freertos_risc_v_trap_handler>:
        if(test_pc_i == 32'h8000_7200 && !finish && !context_switch_flag) begin 
        // if(test_pc_i == 32'h8000_7600 && !finish) begin 
            context_switch_flag <= 1;
            context_switch_cnt <= context_switch_cnt + 1;
        end
        //in processed_source 800073e8:	30200073          	mret

        //rtos_run2
        //800077e8:	30200073          	mret
        if(test_pc_i == 32'h8000_73e8) begin 
        // if(test_pc_i == 32'h8000_77e8) begin 
            context_switch_flag <= 0;
        end

        if(test_pc_i == 32'h8000_163c) begin 
            done <= 1;
        end
    end
end

//---------------
//sync and async overhead
// reg sync_flag, async_flag;
// (* mark_debug = "true" *) reg [16-1:0] sync_cycle;
// (* mark_debug = "true" *) reg [16-1:0] async_cycle;
// (* mark_debug = "true" *) reg [16-1:0] sync_cnt;
// (* mark_debug = "true" *) reg [16-1:0] async_cnt;

// always @(posedge clk_i ) begin
//     if(rst_i)begin
//         sync_flag <= 0;
//         async_flag <= 0;
//     end
//     else begin
//         //80007520 <handle_synchronous>
//         if (context_switch_flag == 1 && pc_i == 32'h8000_7520 && !finish) begin
//             sync_flag <= 1;
//             async_flag <= 0;
//             sync_cnt <= sync_cnt + 1;
//         end
//         //8000749c <handle_asynchronous>
//         else if (context_switch_flag == 1 && pc_i == 32'h8000_749c && !finish) begin
//             sync_flag <= 0;
//             async_flag <= 1;
//             async_cnt <= async_cnt + 1;
//         end
//         // 80007558 <processed_source>
//         else if (pc_i == 32'h8000_7558) begin
//             sync_flag <= 0;
//             async_flag <= 0;
//         end
//     end
// end

// always @(posedge clk_i ) begin
//     if(rst_i)begin
//         sync_cycle <= 0;
//         async_cycle <= 0;
//     end
//     else begin
//         if (async_flag) async_cycle <= async_cycle + 1;
//         if (sync_flag) sync_cycle <= sync_cycle + 1;
//     end
// end



`ifdef USING_SEMAPHORE
// -------------semaphore take give overhead------------
(* mark_debug = "true" *) reg [32-1:0] xsemaphoreTake_cycle;
(* mark_debug = "true" *) reg [32-1:0] xsemaphoreTake_cnt;
(* mark_debug = "true" *) reg [32-1:0] xsemaphoreGive_cycle;
(* mark_debug = "true" *) reg [32-1:0] xsemaphoreGive_cnt;
reg  mutex_take_flag, mutex_give_flag;


//#define xSemaphoreTake(xSemaphore,xBlockTime) xQueueSemaphoreTake( ( xSemaphore ), ( xBlockTime ) )
//#define xSemaphoreGive(xSemaphore) xQueueGenericSend( ( QueueHandle_t ) ( xSemaphore ), NULL, semGIVE_BLOCK_TIME, queueSEND_TO_BACK )
always @(posedge clk_i ) begin
    if(rst_i)begin
        mutex_take_flag <= 0;
        mutex_give_flag <= 0;
        xsemaphoreTake_cnt <= 0;
        xsemaphoreGive_cnt <= 0;
    end
    else begin
        //800030d4 <xQueueSemaphoreTake>

        //rtos_run2
        //8000350c <xQueueSemaphoreTake>:
        if(test_pc_i ==  32'h8000_30d4 && !finish && !mutex_take_flag) begin 
        // if(pc_i ==  32'h8000_350c && !finish) begin 
            mutex_take_flag <= 1;
            xsemaphoreTake_cnt <= xsemaphoreTake_cnt + 1;
        end
        //80003204:	00008067          	ret

        //rtos_run2
        //8000363c:	00008067          	ret
        if(mutex_take_flag && test_pc_i == 32'h8000_3204) begin
        // if(mutex_take_flag && pc_i == 32'h8000_363c) begin
            mutex_take_flag <= 0;
        end
        // 80002a34 <xQueueGenericSend>

        //rtos_run2
        //80002e6c <xQueueGenericSend>:
        if(test_pc_i == 32'h8000_2a34 && !finish && !mutex_give_flag) begin
        // if(pc_i == 32'h8000_2e6c && !finish) begin
            mutex_give_flag <= 1;
            xsemaphoreGive_cnt <= xsemaphoreGive_cnt + 1;
        end
        //80002be8:	00008067          	ret

        //rtos_run2
        //80003020:	00008067          	ret
        if(mutex_give_flag && test_pc_i == 32'h8000_2be8) begin
        // if(mutex_give_flag && pc_i == 32'h8000_3020) begin
            mutex_give_flag <= 0;
        end
    end
end

always @(posedge clk_i ) begin
    if(rst_i)begin
        xsemaphoreTake_cycle <= 0;
        xsemaphoreGive_cycle <= 0;
    end
    else begin
        
        if (mutex_take_flag && !finish) xsemaphoreTake_cycle <= xsemaphoreTake_cycle + 1; 
        
        if(mutex_give_flag && !finish) xsemaphoreGive_cycle <= xsemaphoreGive_cycle + 1; 
    end
end
`endif

//---------------
//basic enter and exit critical overhead

(* mark_debug = "true" *) reg [32-1:0] EnterCritical_cycle;
(* mark_debug = "true" *) reg [32-1:0] EnterCritical_cnt;
(* mark_debug = "true" *) reg [32-1:0] ExitCritical_cycle;
(* mark_debug = "true" *) reg [32-1:0] ExitCritical_cnt;

reg enter_critical_flag, exit_critical_flag;

always @(posedge clk_i ) begin
    if(rst_i)begin
        enter_critical_flag <= 0;
        exit_critical_flag <= 0;
        EnterCritical_cnt <= 0;
        ExitCritical_cnt <= 0;
    end
    else begin
        //80004b48 <vTaskEnterCritical>
        //80004af4 <vTaskEnterCritical>:

        //rtos_run2
        //80004f80 <vTaskEnterCritical>:
        `ifdef USING_SEMAPHORE
        if(test_pc_i == 32'h8000_4b48 && !enter_critical_flag&& !finish) 
        `else 
        if(test_pc_i == 32'h8000_4af4&& !enter_critical_flag&& !finish) 
        `endif
        // if (pc_i == 32'h8000_4f80)
        begin
            enter_critical_flag <= 1;
            EnterCritical_cnt <= EnterCritical_cnt + 1;
        end
        //80004b70:	00008067          	ret
        //80004b1c:	00008067          	ret

        //rtos_run2
        //80004fa8:	00008067          	ret
        `ifdef USING_SEMAPHORE
        if(enter_critical_flag && test_pc_i == 32'h8000_4b70)
        `else
        if(enter_critical_flag && test_pc_i == 32'h8000_4b1c) 
        `endif
        // if(enter_critical_flag && pc_i == 32'h8000_4fa8)
        begin
            enter_critical_flag <= 0;
        end
        //80004b74 <vTaskExitCritical>:
        //80004b20 <vTaskExitCritical>:

        //rtos_run2
        //80004fac <vTaskExitCritical>:
        `ifdef USING_SEMAPHORE
        if(test_pc_i == 32'h8000_4b7c && exit_critical_flag == 0&& !finish)
        `else
        if(test_pc_i == 32'h8000_4b28 && exit_critical_flag == 0&& !finish) 
        `endif
        // if (pc_i == 32'h8000_4fb0 && exit_critical_flag == 0)
        begin
            exit_critical_flag <= 1;
            ExitCritical_cnt <= ExitCritical_cnt + 1;
        end
        //80004bb0:	00008067          	ret
        //80004b5c:	00008067          	ret

        //rtos_run2
        //80004fe8:	00008067          	ret
        `ifdef USING_SEMAPHORE
        if(exit_critical_flag && test_pc_i == 32'h8000_4bb0)
        `else
        if(exit_critical_flag && test_pc_i == 32'h8000_4b5c) 
        `endif
        // if (exit_critical_flag && pc_i == 32'h8000_4fe8)
        begin
            exit_critical_flag <= 0;
        end
    end
end


always @(posedge clk_i ) begin
    if(rst_i)begin
        EnterCritical_cycle <= 0;
        ExitCritical_cycle <= 0;
    end
    else begin
        if (enter_critical_flag && !finish) EnterCritical_cycle <= EnterCritical_cycle + 1; 
        if(exit_critical_flag && !finish) ExitCritical_cycle <= ExitCritical_cycle + 1; 
    end
end

// (* mark_debug = "true" *) reg [32-1:0] test_if_async_cycle;
// (* mark_debug = "true" *) reg [32-1:0] handle_async_cycle;
// (* mark_debug = "true" *) reg [32-1:0] test_if_external_cycle;
// (* mark_debug = "true" *) reg [32-1:0] xTaskIncrementTick_cycle;
// (* mark_debug = "true" *) reg [32-1:0] vTaskSwitchContext_cycle;
// (* mark_debug = "true" *) reg [32-1:0] as_yet_unhandled_cycle;
// (* mark_debug = "true" *) reg [32-1:0] handle_sync_cycle;
// (* mark_debug = "true" *) reg [32-1:0] test_if_environment_cycle;
// (* mark_debug = "true" *) reg [32-1:0] is_expection_cycle;


// always @(posedge clk_i ) begin
//     if(rst_i)begin
//         test_if_async_cycle <= 0;
//         handle_async_cycle <= 0;
//         test_if_external_cycle <= 0;
//         xTaskIncrementTick_cycle <= 0;
//         vTaskSwitchContext_cycle <= 0;
//         as_yet_unhandled_cycle <= 0;
//         handle_sync_cycle <= 0;
//         test_if_environment_cycle <= 0;
//         is_expection_cycle <= 0;
//     end
//     else begin
//         if (!finish) begin
//             //80007490 <test_if_asynchronous>
//             if (pc_i >= 32'h8000_7490 && pc_i <= 32'h8000_7498 ) test_if_async_cycle <= test_if_async_cycle + 1;
//             //8000749c <handle_asynchronous>
//             if (pc_i >= 32'h8000_749c && pc_i <= 32'h8000_7504 ) handle_async_cycle <= handle_async_cycle + 1;
//             //80007508 <test_if_external_interrupt>
//             if (pc_i >= 32'h8000_7508 && pc_i <= 32'h8000_751c ) test_if_external_cycle <= test_if_external_cycle + 1;
//             //80004458 <xTaskIncrementTick>
//             if (pc_i >= 32'h8000_4458 && pc_i <= 32'h8000_4490 ) xTaskIncrementTick_cycle <= xTaskIncrementTick_cycle + 1;
//             //80004494 <vTaskSwitchContext>
//             if (pc_i >= 32'h8000_4494 && pc_i <= 32'h8000_44b0 ) vTaskSwitchContext_cycle <= vTaskSwitchContext_cycle + 1;
//             //80007550 <as_yet_unhandled>
//             if (pc_i >= 32'h8000_7550 && pc_i <= 32'h8000_7554 ) as_yet_unhandled_cycle <= as_yet_unhandled_cycle + 1;
//             //80007520 <handle_synchronous>
//             if (pc_i >= 32'h8000_7520 && pc_i <= 32'h8000_7524 ) handle_sync_cycle <= handle_sync_cycle + 1;
//             //80007528 <test_if_environment_call>
//             if (pc_i >= 32'h8000_7528 && pc_i <= 32'h8000_753c ) test_if_environment_cycle <= test_if_environment_cycle + 1;
//             //80007540 <is_expection>
//             if (pc_i >= 32'h8000_7540 && pc_i <= 32'h8000_754c ) is_expection_cycle <= is_expection_cycle + 1;
//         end
//     end
// end


endmodule
