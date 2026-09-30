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

// Module wrapper for the GVSOC reference model (ISS=GVSOC), the counterpart of
// uvmt_cv32e40p_imperas_dv_wrap. The RVFI rows drive the RVVI trace interface, and
// rvvi_trace2api of the GVSOC bridge steps the model on each row. The bridge compares PC,
// instruction, trap, debug mode, GPRs, FPRs and the CSRs enabled in ref_init, and checks
// the stores of the model against the data bus writes. The model takes interrupts and
// debug entries by itself, from the inputs that uvmt_cv32e40p_gvsoc_decision_monitor
// reports.
//
// Unlike the ImperasDV wrap, this one takes the new value of a CSR written by a trap from
// wdata, starts with csr_wb set, drives rvvi.debug_mode, compares instret and
// mhpmevent3..31, and pushes no nets.

`ifndef __UVMT_CV32E40P_GVSOC_WRAP_SV__
`define __UVMT_CV32E40P_GVSOC_WRAP_SV__

`define GVSOC_RVFI dut_wrap.cv32e40p_tb_wrapper_i.rvfi_i
`define GVSOC_CORE dut_wrap.cv32e40p_tb_wrapper_i.cv32e40p_top_i.core_i

// Drive one CSR of the row, with its csr_wb bit set while the value is new. The bit starts
// set so that the first row reports every CSR, and it is cleared with a nonblocking
// assignment so that the row at that edge still sees it set.
`define GVSOC_RVVI_CSR_VALUE(CSR_ADDR, WB, VALUE) \
    bit WB = 1; \
    assign rvvi.csr[0][0][CSR_ADDR]    = VALUE; \
    assign rvvi.csr_wb[0][0][CSR_ADDR] = WB; \
    always @(rvvi.csr[0][0][CSR_ADDR]) WB = 1; \
    always @(posedge rvvi.clk) if (`GVSOC_RVFI.rvfi_valid && WB) WB <= 0;

// CSR = (wdata & wmask) | (rdata & ~wmask)
`define GVSOC_RVVI_CSR(CSR_ADDR, CSR_NAME) \
    `GVSOC_RVVI_CSR_VALUE(CSR_ADDR, csr_``CSR_NAME``_wb, \
        (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wdata &  `GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask) | \
        (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_rdata & ~`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask))

// The FIELD bits of a CSR that a trap can write. The tracer reports a write by a trap with
// wmask 0 and the new value in wdata, so wdata is taken when wmask has no bit in FIELD.
`define GVSOC_RVVI_TRAP_FIELD(CSR_NAME, FIELD) \
    (((`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask & (FIELD)) == 32'h0) ? `GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wdata : \
     (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wdata &  `GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask) | \
     (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_rdata & ~`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask)) & (FIELD)

`define GVSOC_RVVI_TRAP_CSR(CSR_ADDR, CSR_NAME) \
    `GVSOC_RVVI_CSR_VALUE(CSR_ADDR, csr_``CSR_NAME``_wb, `GVSOC_RVVI_TRAP_FIELD(CSR_NAME, 32'hFFFF_FFFF))

// Element ID of an RVFI CSR array (mhpmevent, mhpmcounter, dscratch, tdata).
`define GVSOC_RVVI_CSR_VEC(CSR_ADDR, CSR_NAME, CSR_ID) \
    `GVSOC_RVVI_CSR_VALUE(CSR_ADDR, csr_wb, \
        (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wdata[CSR_ID] &  `GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask[CSR_ID]) | \
        (`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_rdata[CSR_ID] & ~`GVSOC_RVFI.rvfi_csr_``CSR_NAME``_wmask[CSR_ID]))

module uvmt_cv32e40p_gvsoc_wrap
    import uvm_pkg::*;
    import rvviApiPkg::*;
    #(
        parameter FPU = 0   // compare the FP status CSRs
    )
    (
        rvviTrace rvvi
    );

    localparam bit [11:0] CSR_FFLAGS        = 12'h001, CSR_FRM           = 12'h002,
                          CSR_FCSR          = 12'h003, CSR_MSTATUS       = 12'h300,
                          CSR_MISA          = 12'h301, CSR_MIE           = 12'h304,
                          CSR_MTVEC         = 12'h305, CSR_MCOUNTINHIBIT = 12'h320,
                          CSR_MHPMEVENT     = 12'h320, CSR_MSCRATCH      = 12'h340,
                          CSR_MEPC          = 12'h341, CSR_MCAUSE        = 12'h342,
                          CSR_MTVAL         = 12'h343, CSR_MIP           = 12'h344,
                          CSR_TSELECT       = 12'h7A0, CSR_TDATA1        = 12'h7A1,
                          CSR_TDATA2        = 12'h7A2, CSR_TDATA3        = 12'h7A3,
                          CSR_TINFO         = 12'h7A4, CSR_DCSR          = 12'h7B0,
                          CSR_DPC           = 12'h7B1, CSR_DSCRATCH0     = 12'h7B2,
                          CSR_DSCRATCH1     = 12'h7B3, CSR_MCYCLE        = 12'hB00,
                          CSR_MINSTRET      = 12'hB02, CSR_MHPMCOUNTER   = 12'hB00,
                          CSR_MCYCLEH       = 12'hB80, CSR_MINSTRETH     = 12'hB82,
                          CSR_MHPMCOUNTERH  = 12'hB80, CSR_CYCLE         = 12'hC00,
                          CSR_INSTRET       = 12'hC02, CSR_CYCLEH        = 12'hC80,
                          CSR_INSTRETH      = 12'hC82, CSR_LPSTART0      = 12'hCC0,
                          CSR_LPEND0        = 12'hCC1, CSR_LPCOUNT0      = 12'hCC2,
                          CSR_LPSTART1      = 12'hCC4, CSR_LPEND1        = 12'hCC5,
                          CSR_LPCOUNT1      = 12'hCC6, CSR_MVENDORID     = 12'hF11,
                          CSR_MARCHID       = 12'hF12, CSR_MHARTID       = 12'hF14;
    localparam bit [31:0] MSTATUS_FS_SD     = 32'h8000_6000;

    string info_tag = "GVSOC_wrap";

    // The decision monitor reports the interrupt and debug nets, so rvvi_trace2api pushes none.
    rvvi_trace2api #(
        .NHART  (1),
        .RETIRE (1),
        .NETS   (0)
    ) gvsoc_sync (
        .rvvi (rvvi)
    );

    uvmt_cv32e40p_gvsoc_decision_monitor decision_monitor (
        .rvvi (rvvi)
    );

    ////////////////////////////////////////////////////////////////////////////
    // Assign the RVVI trace from RVFI
    ////////////////////////////////////////////////////////////////////////////
    assign rvvi.clk              = `GVSOC_RVFI.clk_i;
    assign rvvi.valid[0][0]      = `GVSOC_RVFI.rvfi_valid;
    assign rvvi.order[0][0]      = `GVSOC_RVFI.rvfi_order;
    assign rvvi.insn[0][0]       = `GVSOC_RVFI.rvfi_insn;
    assign rvvi.trap[0][0]       = `GVSOC_RVFI.rvfi_trap.trap;
    assign rvvi.intr[0][0]       = `GVSOC_RVFI.rvfi_intr.intr;
    assign rvvi.mode[0][0]       = `GVSOC_RVFI.rvfi_mode;
    assign rvvi.debug_mode[0][0] = `GVSOC_RVFI.rvfi_dbg_mode;
    assign rvvi.ixl[0][0]        = `GVSOC_RVFI.rvfi_ixl;
    assign rvvi.pc_rdata[0][0]   = `GVSOC_RVFI.rvfi_pc_rdata;

    // The tracer gives FS and SD a write mask of their own, apart from the other mstatus
    // bits, so apply the trap rule to each part.
    `GVSOC_RVVI_CSR_VALUE(CSR_MSTATUS, csr_mstatus_wb,
        `GVSOC_RVVI_TRAP_FIELD(mstatus, MSTATUS_FS_SD) | `GVSOC_RVVI_TRAP_FIELD(mstatus, ~MSTATUS_FS_SD))
    `GVSOC_RVVI_CSR     (CSR_MISA,          misa         )
    `GVSOC_RVVI_CSR     (CSR_MIE,           mie          )
    `GVSOC_RVVI_CSR     (CSR_MTVEC,         mtvec        )
    `GVSOC_RVVI_CSR     (CSR_MCOUNTINHIBIT, mcountinhibit)
    `GVSOC_RVVI_CSR     (CSR_MSCRATCH,      mscratch     )
    `GVSOC_RVVI_TRAP_CSR(CSR_MEPC,          mepc         )
    `GVSOC_RVVI_TRAP_CSR(CSR_MCAUSE,        mcause       )
    `GVSOC_RVVI_TRAP_CSR(CSR_MTVAL,         mtval        )
    `GVSOC_RVVI_CSR     (CSR_MIP,           mip          )
    `GVSOC_RVVI_CSR     (CSR_MINSTRET,      minstret     )
    `GVSOC_RVVI_CSR     (CSR_MINSTRETH,     minstreth    )
    `GVSOC_RVVI_CSR     (CSR_INSTRET,       instret      )
    `GVSOC_RVVI_CSR     (CSR_INSTRETH,      instreth     )
    `GVSOC_RVVI_CSR     (CSR_MVENDORID,     mvendorid    )
    `GVSOC_RVVI_CSR     (CSR_MARCHID,       marchid      )
    `GVSOC_RVVI_CSR     (CSR_MHARTID,       mhartid      )
    `GVSOC_RVVI_CSR     (CSR_DCSR,          dcsr         )
    `GVSOC_RVVI_CSR     (CSR_DPC,           dpc          )
    `GVSOC_RVVI_CSR     (CSR_TINFO,         tinfo        )
    `GVSOC_RVVI_CSR     (CSR_FFLAGS,        fflags       )
    `GVSOC_RVVI_CSR     (CSR_FRM,           frm          )
    `GVSOC_RVVI_CSR     (CSR_FCSR,          fcsr         )
    `GVSOC_RVVI_CSR     (CSR_LPSTART0,      lpstart0     )
    `GVSOC_RVVI_CSR     (CSR_LPEND0,        lpend0       )
    `GVSOC_RVVI_CSR     (CSR_LPCOUNT0,      lpcount0     )
    `GVSOC_RVVI_CSR     (CSR_LPSTART1,      lpstart1     )
    `GVSOC_RVVI_CSR     (CSR_LPEND1,        lpend1       )
    `GVSOC_RVVI_CSR     (CSR_LPCOUNT1,      lpcount1     )

    for (genvar n = 0; n < 2; n++) begin : gen_dscratch
        `GVSOC_RVVI_CSR_VEC(CSR_DSCRATCH0 + n, dscratch, n)
    end
    for (genvar n = 1; n < 3; n++) begin : gen_tdata
        `GVSOC_RVVI_CSR_VEC(CSR_TSELECT + n, tdata, n)
    end
    for (genvar n = 3; n < 32; n++) begin : gen_mhpmevent
        `GVSOC_RVVI_CSR_VEC(CSR_MHPMEVENT + n, mhpmevent, n)
    end
    for (genvar n = 3; n < 32; n++) begin : gen_mhpmcounter
        `GVSOC_RVVI_CSR_VEC(CSR_MHPMCOUNTER + n, mhpmcounter, n)
    end
    for (genvar n = 3; n < 32; n++) begin : gen_mhpmcounterh
        `GVSOC_RVVI_CSR_VEC(CSR_MHPMCOUNTERH + n, mhpmcounterh, n)
    end

    // GPRs and FPRs written by the row (RVFI reports up to two destinations).
    for (genvar i = 0; i < 32; i++) begin : gen_regs
        assign rvvi.x_wdata[0][0][i] =
            (i != 0 && `GVSOC_RVFI.rvfi_rd_addr[1] == 5'(i)) ? `GVSOC_RVFI.rvfi_rd_wdata[1] :
            (i != 0 && `GVSOC_RVFI.rvfi_rd_addr[0] == 5'(i)) ? `GVSOC_RVFI.rvfi_rd_wdata[0] : 32'h0;
        assign rvvi.f_wdata[0][0][i] =
            (`GVSOC_RVFI.rvfi_frd_wvalid[1] && `GVSOC_RVFI.rvfi_frd_addr[1] == 5'(i)) ? `GVSOC_RVFI.rvfi_frd_wdata[1] :
            (`GVSOC_RVFI.rvfi_frd_wvalid[0] && `GVSOC_RVFI.rvfi_frd_addr[0] == 5'(i)) ? `GVSOC_RVFI.rvfi_frd_wdata[0] : 32'h0;
    end
    assign rvvi.x_wb[0][0] = (1 << `GVSOC_RVFI.rvfi_rd_addr[0]) | (1 << `GVSOC_RVFI.rvfi_rd_addr[1]);
    assign rvvi.f_wb[0][0] = (`GVSOC_RVFI.rvfi_frd_wvalid[0] << `GVSOC_RVFI.rvfi_frd_addr[0]) |
                             (`GVSOC_RVFI.rvfi_frd_wvalid[1] << `GVSOC_RVFI.rvfi_frd_addr[1]);

    ////////////////////////////////////////////////////////////////////////////
    // Data bus writes
    ////////////////////////////////////////////////////////////////////////////
    // Report each write accepted on the data port, with its byte enables on the word address.
    always @(posedge `GVSOC_CORE.clk_i) begin
        if (`GVSOC_CORE.data_req_o && `GVSOC_CORE.data_gnt_i && `GVSOC_CORE.data_we_o)
            rvviDutBusWrite(0, {`GVSOC_CORE.data_addr_o[31:2], 2'b00}, `GVSOC_CORE.data_wdata_o,
                            `GVSOC_CORE.data_be_o);
    end

    // Shut the reference down when the run phase ends, before the end-of-test metrics are
    // read, so that a store of the model without its bus write counts as a mismatch.
    initial begin
        automatic uvm_phase extract_phase =
            uvm_domain::get_common_domain().find(uvm_extract_phase::get());
        extract_phase.wait_for_state(UVM_PHASE_STARTED);
        void'(rvviRefShutdown());
    end

    ////////////////////////////////////////////////////////////////////////////
    // REF control
    ////////////////////////////////////////////////////////////////////////////
    task automatic ref_init;
        string     test_program_elf;
        bit [31:0] hart_id = 32'h0;
        bit [63:0] mtvec_addr;
        bit [11:0] compared[] = '{
            CSR_MISA, CSR_MSTATUS, CSR_MIE, CSR_MTVEC, CSR_MCOUNTINHIBIT, CSR_MSCRATCH,
            CSR_MEPC, CSR_MCAUSE, CSR_MTVAL,
            CSR_DCSR, CSR_DPC, CSR_DSCRATCH0, CSR_DSCRATCH1,
            CSR_MINSTRET, CSR_MINSTRETH, CSR_INSTRET, CSR_INSTRETH,
            CSR_TDATA1, CSR_TDATA2, CSR_TINFO,
            CSR_MVENDORID, CSR_MARCHID, CSR_MHARTID,
            CSR_LPSTART0, CSR_LPEND0, CSR_LPCOUNT0, CSR_LPSTART1, CSR_LPEND1, CSR_LPCOUNT1};

        if (!rvviVersionCheck(RVVI_API_VERSION))
            `uvm_fatal(info_tag, $sformatf("Expecting RVVI API version %0d.", RVVI_API_VERSION))

        if (!$value$plusargs("elf_file=%s", test_program_elf))
            `uvm_fatal(info_tag, "No elf_file plusarg specified")
        `uvm_info(info_tag, $sformatf("Loading ELF: %0s", test_program_elf), UVM_NONE)
        if (!rvviRefInit(test_program_elf))
            `uvm_fatal(info_tag, $sformatf("rvviRefInit failed: %s", rvviErrorGet()))

        // These CSRs depend on timing, so a read of them takes the value the RTL read.
        void'(rvviRefCsrSetVolatile(hart_id, CSR_CYCLE));
        void'(rvviRefCsrSetVolatile(hart_id, CSR_CYCLEH));
        void'(rvviRefCsrSetVolatile(hart_id, CSR_MCYCLE));
        void'(rvviRefCsrSetVolatile(hart_id, CSR_MCYCLEH));
        void'(rvviRefCsrSetVolatile(hart_id, CSR_MIP));
        for (int n = 3; n < 32; n++) begin
            void'(rvviRefCsrSetVolatile(hart_id, CSR_MHPMCOUNTER  + n));
            void'(rvviRefCsrSetVolatile(hart_id, CSR_MHPMCOUNTERH + n));
        end

        // A load from the virtual peripherals of the testbench, such as the random number
        // generator and the cycle counter, also takes the value the RTL loaded.
        void'(rvviRefMemorySetVolatile('h15001000, 'h15001007));

        // The event selectors change only on software writes, so they are compared too.
        // tselect and tdata3 read 0 and RVFI does not report them, so a read of them is
        // checked through its destination register.
        foreach (compared[i])
            void'(rvviRefCsrCompareEnable(hart_id, compared[i], RVVI_TRUE));
        for (int n = 3; n < 32; n++)
            void'(rvviRefCsrCompareEnable(hart_id, CSR_MHPMEVENT + n, RVVI_TRUE));
        if (FPU) begin
            void'(rvviRefCsrCompareEnable(hart_id, CSR_FFLAGS, RVVI_TRUE));
            void'(rvviRefCsrCompareEnable(hart_id, CSR_FRM,    RVVI_TRUE));
            void'(rvviRefCsrCompareEnable(hart_id, CSR_FCSR,   RVVI_TRUE));
        end

        // At reset mtvec comes from the mtvec_addr_i input of the core, set by the test plusarg.
        mtvec_addr = 64'h1;
        void'($value$plusargs("mtvec_addr=%0x", mtvec_addr));
        `uvm_info(info_tag, $sformatf("mtvec set to 0x%08x", mtvec_addr), UVM_NONE)
        rvviRefCsrSet(hart_id, CSR_MTVEC, mtvec_addr);

        `uvm_info(info_tag, "GVSOC ref_init complete", UVM_NONE)
    endtask

endmodule

`undef GVSOC_RVVI_CSR_VEC
`undef GVSOC_RVVI_TRAP_CSR
`undef GVSOC_RVVI_TRAP_FIELD
`undef GVSOC_RVVI_CSR
`undef GVSOC_RVVI_CSR_VALUE
`undef GVSOC_CORE
`undef GVSOC_RVFI

`endif // __UVMT_CV32E40P_GVSOC_WRAP_SV__
