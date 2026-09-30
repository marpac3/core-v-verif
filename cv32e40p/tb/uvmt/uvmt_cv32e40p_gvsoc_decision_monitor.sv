//
// Copyright 2026 Fondazione Chips-IT
//
// Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://solderpad.org/licenses/
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.0
//

// Reports to the reference model what it needs to take interrupts and debug requests by
// itself, through the decision point extension of the RVVI API (rvviDecisionApiPkg):
// - the interrupt and debug request pins, with rvviRefNetSet;
// - the samples of these pins by the RTL, irq_q on the gated clock and debug_req_q on the
//   free-running clock, with rvviRefNetGroupSample;
// - the cycles where the CV32E40P controller evaluates them, with rvviRefDecisionPoint.
// The model is never told what the RTL decided.
//
// A decision point names the instruction it precedes by its RVFI order. Each RVFI row must
// be the instruction issued at that place, otherwise the model would decide before the
// wrong instruction.

`ifndef __UVMT_CV32E40P_GVSOC_DECISION_MONITOR_SV__
`define __UVMT_CV32E40P_GVSOC_DECISION_MONITOR_SV__

`define GVSOC_DM_CORE uvmt_cv32e40p_tb.dut_wrap.cv32e40p_tb_wrapper_i.cv32e40p_top_i.core_i
`define GVSOC_DM_CTRL `GVSOC_DM_CORE.id_stage_i.controller_i

module uvmt_cv32e40p_gvsoc_decision_monitor
    import uvm_pkg::*;
    import cv32e40p_pkg::*;
    import rvviApiPkg::*;
    import rvviDecisionApiPkg::*;
(
    rvviTrace rvvi
);

    localparam logic [31:0] IRQ_MASK    = 32'hFFFF0888;
    localparam int          GROUP_IRQ   = 1;
    localparam int          GROUP_DEBUG = 2;

    string           info_tag      = "GVSOC_decision";
    longint          net_index[33];          // net of each irq_i bit, and of haltreq at 32
    logic [31:0]     irq_reported  = '0;
    logic            halt_reported = 1'b0;
    logic [31:0]     irq_sampled   = '0;
    logic            halt_sampled  = 1'b0;   // debug_req_q as loaded by the reported samples
    bit              started       = 0;

    longint unsigned issued     = 0;
    bit              id_issued  = 0;         // the instruction in ID has been issued
    logic [31:0]     id_pc      = '0;
    longint unsigned cycle      = 0;
    bit              new_input  = 1;         // a pin or sample since the last decision point
    int              last_kind  = -1;
    longint unsigned last_order = 0;

    logic [31:0]     issued_pc[$];           // pc_id of the issued instructions not retired yet
    longint unsigned rows       = 0;
    longint unsigned decisions  = 0;

    function automatic void start();
        string names[int];
        names[3]  = "MSWInterrupt";
        names[7]  = "MTimerInterrupt";
        names[11] = "MExternalInterrupt";
        for (int i = 16; i < 32; i++) names[i] = $sformatf("LocalInterrupt%0d", i - 16);
        foreach (names[i]) begin
            net_index[i] = rvviRefNetIndexGet(names[i]);
            rvviRefNetGroupSet(net_index[i], GROUP_IRQ);
        end
        net_index[32] = rvviRefNetIndexGet("haltreq");
        rvviRefNetGroupSet(net_index[32], GROUP_DEBUG);
        started = 1;
    endfunction

    function automatic void report_pins();
        logic [31:0] irq  = `GVSOC_DM_CORE.irq_i & IRQ_MASK;
        logic        halt = `GVSOC_DM_CORE.debug_req_i;
        for (int i = 0; i < 32; i++)
            if (irq[i] !== irq_reported[i])
                rvviRefNetSet(net_index[i], irq[i], cycle);
        if (halt !== halt_reported)
            rvviRefNetSet(net_index[32], halt, cycle);
        if (irq !== irq_reported || halt !== halt_reported)
            new_input = 1;
        irq_reported  = irq;
        halt_reported = halt;
    endfunction

    // Report a decision point. A point of the same kind before the same instruction, with no
    // new input since, would decide the same and is skipped unless always_report is set.
    function automatic void decision(rvviDecisionE kind, bit always_report);
        longint unsigned order = issued + 1;
        if (!always_report && !new_input && kind == last_kind && order == last_order)
            return;
        if (!rvviRefDecisionPoint(0, kind, order))
            `uvm_fatal(info_tag, $sformatf("%s before instruction %0d rejected: %s",
                                           kind.name(), order, rvviErrorGet()))
        decisions++;
        new_input  = 0;
        last_kind  = kind;
        last_order = order;
    endfunction

    // Report the decision point of the cycle that just ended, if the controller made one.
    function automatic void report_decision();
        case (ctrl_state_e'(`GVSOC_DM_CTRL.ctrl_fsm_cs))
            BOOT_SET:
                decision(RVVI_DECISION_BOOT, 1);
            FIRST_FETCH:
                decision(RVVI_DECISION_FIRST_FETCH, 1);
            SLEEP:
                decision(RVVI_DECISION_SLEEP, 0);
            FLUSH_WB:
                if (`GVSOC_DM_CTRL.wfi_i)
                    decision(RVVI_DECISION_SLEEP, 1);
            DECODE:
                if (`GVSOC_DM_CTRL.instr_valid_i && !`GVSOC_DM_CTRL.branch_taken_ex_i &&
                    !`GVSOC_DM_CTRL.data_err_i && !`GVSOC_DM_CTRL.is_fetch_failed_i)
                    decision(RVVI_DECISION_DISPATCH, 0);
            DECODE_HWLOOP:
                if (`GVSOC_DM_CTRL.instr_valid_i)
                    decision(RVVI_DECISION_DISPATCH, 0);
            default: ;
        endcase
    endfunction

    function automatic void issue();
        issued++;
        issued_pc.push_back(`GVSOC_DM_CORE.id_stage_i.pc_id_i);
        id_issued = 1;
    endfunction

    // Count as issued the instructions that RVFI reports without an ID handshake:
    // - an ebreak that enters debug mode through dcsr.ebreakm, unless a halt request
    //   (debug_req_entry_q) killed it in ID;
    // - an ebreak in debug mode, which goes from DECODE to DBG_FLUSH even when ID is stalled;
    // - an illegal instruction in DECODE_HWLOOP, which goes to FLUSH_EX with ID halted
    //   (halt_id_o).
    function automatic void track_issue();
        if (`GVSOC_DM_CORE.id_stage_i.pc_id_i !== id_pc) begin
            id_pc     = `GVSOC_DM_CORE.id_stage_i.pc_id_i;
            id_issued = 0;
        end
        if (ctrl_state_e'(`GVSOC_DM_CTRL.ctrl_fsm_cs) == DBG_TAKEN_ID && !id_issued &&
            ((!`GVSOC_DM_CTRL.debug_mode_q && `GVSOC_DM_CTRL.debug_cause_o == DBG_CAUSE_EBREAK &&
              !`GVSOC_DM_CTRL.debug_req_entry_q) ||
             (`GVSOC_DM_CTRL.debug_mode_q && `GVSOC_DM_CTRL.ebrk_insn_i)))
            issue();
        if (ctrl_state_e'(`GVSOC_DM_CTRL.ctrl_fsm_cs) == FLUSH_EX &&
            `GVSOC_DM_CTRL.illegal_insn_q && !id_issued)
            issue();
    endfunction

    function automatic void report_samples();
        // The gated clock rises at this edge only if its enable was latched.
        if (`GVSOC_DM_CORE.sleep_unit_i.core_clock_gate_i.clk_en) begin
            logic [31:0] irq = `GVSOC_DM_CORE.irq_i & IRQ_MASK;
            // Sampling unchanged levels would change nothing in the model, so skip it.
            if (irq !== irq_sampled) begin
                void'(rvviRefNetGroupSample(GROUP_IRQ));
                irq_sampled = irq;
                new_input = 1;
            end
            if (`GVSOC_DM_CTRL.id_valid_i && `GVSOC_DM_CTRL.is_decoding_o)
                issue();
        end
        // debug_req_q loads the pin on the free-running clock while the pin is high or the
        // core is in debug mode (cv32e40p_controller.sv).
        if ((`GVSOC_DM_CORE.debug_req_i || `GVSOC_DM_CTRL.debug_mode_q) &&
            `GVSOC_DM_CORE.debug_req_i !== halt_sampled) begin
            void'(rvviRefNetGroupSample(GROUP_DEBUG));
            halt_sampled = `GVSOC_DM_CORE.debug_req_i;
            new_input = 1;
        end
    endfunction

    // At each clock edge, report the pins that changed and the decision point of the cycle
    // that just ended, then the samples that the RTL takes at this edge.
    always @(posedge `GVSOC_DM_CORE.clk_i) begin
        if (`GVSOC_DM_CORE.rst_ni === 1'b1) begin
            if (!started) start();
            cycle++;
            track_issue();
            report_pins();
            report_decision();
            report_samples();
        end
    end

    // Every RVFI row must come from the next issued instruction, in order.
    always @(posedge rvvi.clk) begin
        if (rvvi.valid[0][0]) begin
            rows++;
            if (rvvi.order[0][0] != rows || issued_pc.size() == 0 ||
                issued_pc[0] != rvvi.pc_rdata[0][0])
                `uvm_fatal(info_tag, $sformatf(
                    "RVFI row %0d (order %0d, pc 0x%08x, insn 0x%08x, trap %0d) does not match issued instruction %0d (pc 0x%08x)",
                    rows, rvvi.order[0][0], rvvi.pc_rdata[0][0], rvvi.insn[0][0], rvvi.trap[0][0],
                    rows, issued_pc.size() ? issued_pc[0] : 32'hx))
            void'(issued_pc.pop_front());
        end
    end

    final
        `uvm_info(info_tag, $sformatf("rows %0d, issued %0d, decision points %0d",
                                      rows, issued, decisions), UVM_LOW)

endmodule

`undef GVSOC_DM_CTRL
`undef GVSOC_DM_CORE

`endif
