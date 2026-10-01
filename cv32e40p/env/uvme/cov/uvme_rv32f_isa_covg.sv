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

// RV32F, Zcf and Zfinx instruction coverage. uvma_isacov decodes RV32IMC only. This
// component decodes the F instructions of its transactions from rvfi.insn and samples
// one covergroup per instruction. RVFI reports integer registers only, so operand values
// are covered for every operand with Zfinx, and only on the GPR side with F.

`ifndef __UVME_RV32F_ISA_COVG_SV__
`define __UVME_RV32F_ISA_COVG_SV__

typedef enum {
  RV32F_UNKNOWN,
  FLW, FSW,
  FMADD_S, FMSUB_S, FNMSUB_S, FNMADD_S,
  FADD_S, FSUB_S, FMUL_S, FDIV_S, FSQRT_S,
  FSGNJ_S, FSGNJN_S, FSGNJX_S, FMIN_S, FMAX_S,
  FCVT_W_S, FCVT_WU_S, FMV_X_W, FEQ_S, FLT_S, FLE_S, FCLASS_S,
  FCVT_S_W, FCVT_S_WU, FMV_W_X,
  C_FLW, C_FSW, C_FLWSP, C_FSWSP
} rv32f_instr_name_t;

// The ten classes of fclass.s, in the order of its result bits
typedef enum {
  FP_NEG_INF, FP_NEG_NORMAL, FP_NEG_SUBNORMAL, FP_NEG_ZERO,
  FP_POS_ZERO, FP_POS_SUBNORMAL, FP_POS_NORMAL, FP_POS_INF,
  FP_SNAN, FP_QNAN
} rv32f_class_t;

typedef enum {
  INT_ZERO,
  INT_POSITIVE,
  INT_NEGATIVE
} rv32f_int_value_t;

typedef struct {
  rv32f_instr_name_t name;
  bit [4:0]          rs1;
  bit [4:0]          rs2;
  bit [4:0]          rs3;
  bit [4:0]          rd;
  bit [2:0]          rm;   // rounding mode field of the instruction
  bit [2:0]          frm;  // fcsr.frm, the rounding mode that rm == DYN selects
  bit [11:0]         imm;  // offset of the loads and stores as encoded, the scaled field for C
  bit [1:0]          mem_addr;
  bit [31:0]         rs1_value;
  bit [31:0]         rs2_value;
  bit [31:0]         rd_value;
  rv32f_class_t      rs1_class;
  rv32f_class_t      rs2_class;
  rv32f_class_t      rs3_class;
  rv32f_class_t      rd_class;
} rv32f_instr_t;

function automatic rv32f_class_t get_rv32f_class(bit [31:0] value);
  bit sign = value[31];

  if (value[30:23] == 8'hFF) begin
    if (value[22:0] == 0)
      return sign ? FP_NEG_INF : FP_POS_INF;
    return value[22] ? FP_QNAN : FP_SNAN;
  end
  if (value[30:23] == 0) begin
    if (value[22:0] == 0)
      return sign ? FP_NEG_ZERO : FP_POS_ZERO;
    return sign ? FP_NEG_SUBNORMAL : FP_POS_SUBNORMAL;
  end
  return sign ? FP_NEG_NORMAL : FP_POS_NORMAL;
endfunction : get_rv32f_class

function automatic rv32f_int_value_t get_rv32f_int_value(bit [31:0] value, bit is_signed);
  if (value == 0)
    return INT_ZERO;
  return (is_signed && value[31]) ? INT_NEGATIVE : INT_POSITIVE;
endfunction : get_rv32f_int_value

function automatic rv32f_instr_name_t decode_rv32f(bit [31:0] insn);
  if (insn[1:0] != 2'b11) begin
    casez (insn[15:0])
      16'b011_???_???_??_???_00: return C_FLW;
      16'b111_???_???_??_???_00: return C_FSW;
      16'b011_?_?????_?????_10:  return C_FLWSP;
      16'b111_??????_?????_10:   return C_FSWSP;
      default:                   return RV32F_UNKNOWN;
    endcase
  end

  casez (insn)
    TB_INS_FLW:     return FLW;
    TB_INS_FSW:     return FSW;
    TB_INS_FMADD:   return FMADD_S;
    TB_INS_FMSUB:   return FMSUB_S;
    TB_INS_FNMSUB:  return FNMSUB_S;
    TB_INS_FNMADD:  return FNMADD_S;
    TB_INS_FADD:    return FADD_S;
    TB_INS_FSUB:    return FSUB_S;
    TB_INS_FMUL:    return FMUL_S;
    TB_INS_FDIV:    return FDIV_S;
    TB_INS_FSQRT:   return FSQRT_S;
    TB_INS_FSGNJS:  return FSGNJ_S;
    TB_INS_FSGNJNS: return FSGNJN_S;
    TB_INS_FSGNJXS: return FSGNJX_S;
    TB_INS_FMIN:    return FMIN_S;
    TB_INS_FMAX:    return FMAX_S;
    TB_INS_FCVTWS:  return FCVT_W_S;
    TB_INS_FCVTWUS: return FCVT_WU_S;
    TB_INS_FMVXS:   return FMV_X_W;
    TB_INS_FEQS:    return FEQ_S;
    TB_INS_FLTS:    return FLT_S;
    TB_INS_FLES:    return FLE_S;
    TB_INS_FCLASS:  return FCLASS_S;
    TB_INS_FCVTSW:  return FCVT_S_W;
    TB_INS_FCVTSWU: return FCVT_S_WU;
    TB_INS_FMVSX:   return FMV_W_X;
    default:        return RV32F_UNKNOWN;
  endcase
endfunction : decode_rv32f

// Rounding mode of the instructions that have one, static or DYN, and the mode that DYN
// takes from fcsr.frm. The reserved encodings trap.
`define RV32F_CP_RM(ENABLED) \
  cp_rm: coverpoint instr.rm { \
    ignore_bins IGN_OFF = {[0:7]} with (!(ENABLED)); \
    bins RNE = {3'b000}; \
    bins RTZ = {3'b001}; \
    bins RDN = {3'b010}; \
    bins RUP = {3'b011}; \
    bins RMM = {3'b100}; \
    bins DYN = {3'b111}; \
  } \
  cp_frm_dyn: coverpoint instr.frm iff (instr.rm == 3'b111) { \
    ignore_bins IGN_OFF = {[0:7]} with (!(ENABLED)); \
    bins RNE = {3'b000}; \
    bins RTZ = {3'b001}; \
    bins RDN = {3'b010}; \
    bins RUP = {3'b011}; \
    bins RMM = {3'b100}; \
  }

`define RV32F_CP_CLASS(NAME, FIELD, ENABLED) \
  NAME: coverpoint instr.FIELD { \
    ignore_bins IGN_OFF = {[FP_NEG_INF:FP_QNAN]} with (!(ENABLED)); \
  }

// x0 as destination (Zfinx) discards the result
`define RV32F_CP_RD_CLASS(ENABLED) \
  cp_rd_class: coverpoint instr.rd_class iff (instr.rd != 0) { \
    ignore_bins IGN_OFF = {[FP_NEG_INF:FP_QNAN]} with (!(ENABLED)); \
  }

`define RV32F_CP_HAZARD(NAME, RS, ENABLED) \
  NAME: coverpoint instr.rd { \
    ignore_bins IGN_OFF = {[0:31]} with (!(ENABLED)); \
    bins RD[] = {[0:31]} iff (instr.rd == instr.RS); \
  }

// Two FPR sources, FPR destination: fadd.s, fsub.s, fmul.s, fdiv.s, fsgnj*.s, fmin.s, fmax.s
covergroup cg_rv32f_r(string name, bit has_rm, bit values) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, 1)
  `RV32F_CP_HAZARD(cp_rd_rs2_hazard, rs2, 1)

  `RV32F_CP_RM(has_rm)

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, values)
  `RV32F_CP_CLASS(cp_rs2_class, rs2_class, values)
  `RV32F_CP_RD_CLASS(values)
endgroup : cg_rv32f_r

// Three FPR sources, FPR destination: fmadd.s, fmsub.s, fnmsub.s, fnmadd.s
covergroup cg_rv32f_r4(string name, bit values) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2;
  cp_rs3: coverpoint instr.rs3;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, 1)
  `RV32F_CP_HAZARD(cp_rd_rs2_hazard, rs2, 1)
  `RV32F_CP_HAZARD(cp_rd_rs3_hazard, rs3, 1)

  `RV32F_CP_RM(1)

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, values)
  `RV32F_CP_CLASS(cp_rs2_class, rs2_class, values)
  `RV32F_CP_CLASS(cp_rs3_class, rs3_class, values)
  `RV32F_CP_RD_CLASS(values)
endgroup : cg_rv32f_r4

// One FPR source, FPR destination: fsqrt.s
covergroup cg_rv32f_r1(string name, bit values) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, 1)

  `RV32F_CP_RM(1)

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, values)
  `RV32F_CP_RD_CLASS(values)
endgroup : cg_rv32f_r1

// FPR source, GPR destination: fcvt.w.s and fcvt.wu.s. The limits of the result are
// the saturated results of NaN and out-of-range sources.
covergroup cg_rv32f_cvt_to_int(string name, bit is_signed, bit zfinx) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, zfinx)

  `RV32F_CP_RM(1)

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, zfinx)

  cp_rd_value: coverpoint get_rv32f_int_value(instr.rd_value, is_signed) iff (instr.rd != 0) {
    ignore_bins NEG_OFF = {INT_NEGATIVE} with (!is_signed);
  }

  cp_rd_limit: coverpoint instr.rd_value iff (instr.rd != 0) {
    bins MAX  = {32'h7FFF_FFFF};
    bins MIN  = {32'h8000_0000};
    bins UMAX = {32'hFFFF_FFFF};
    ignore_bins SIGNED_OFF   = {32'hFFFF_FFFF} with (is_signed);
    ignore_bins UNSIGNED_OFF = {32'h7FFF_FFFF, 32'h8000_0000} with (!is_signed);
  }
endgroup : cg_rv32f_cvt_to_int

// GPR source, FPR destination: fcvt.s.w and fcvt.s.wu
covergroup cg_rv32f_cvt_from_int(string name, bit is_signed, bit zfinx) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, zfinx)

  `RV32F_CP_RM(1)

  cp_rs1_value: coverpoint get_rv32f_int_value(instr.rs1_value, is_signed) {
    ignore_bins NEG_OFF = {INT_NEGATIVE} with (!is_signed);
  }

  `RV32F_CP_RD_CLASS(zfinx)
endgroup : cg_rv32f_cvt_from_int

// Bit moves between the register files (F only): fmv.x.w and fmv.w.x. The GPR side
// carries the FP value, so its class is covered.
covergroup cg_rv32f_mv(string name, bit to_int) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, !to_int)
  `RV32F_CP_RD_CLASS(to_int)
endgroup : cg_rv32f_mv

// FPR sources, GPR destination: feq.s, flt.s, fle.s
covergroup cg_rv32f_cmp(string name, bit zfinx) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, zfinx)
  `RV32F_CP_HAZARD(cp_rd_rs2_hazard, rs2, zfinx)

  `RV32F_CP_CLASS(cp_rs1_class, rs1_class, zfinx)
  `RV32F_CP_CLASS(cp_rs2_class, rs2_class, zfinx)

  cp_rd_value: coverpoint instr.rd_value iff (instr.rd != 0) {
    bins FALSE = {0};
    bins TRUE  = {1};
  }
endgroup : cg_rv32f_cmp

// fclass.s. Its one-hot result gives the class of the source, so the class is covered
// with F as well.
covergroup cg_rv32f_class(string name, bit zfinx) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;

  `RV32F_CP_HAZARD(cp_rd_rs1_hazard, rs1, zfinx)

  cp_rd_value: coverpoint instr.rd_value iff (instr.rd != 0) {
    bins CLASS[] = {32'h001, 32'h002, 32'h004, 32'h008, 32'h010,
                    32'h020, 32'h040, 32'h080, 32'h100, 32'h200};
  }
endgroup : cg_rv32f_class

// flw and fsw (F only)
covergroup cg_rv32f_ls(string name, bit is_store) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd: coverpoint instr.rd {
    ignore_bins IGN_OFF = {[0:31]} with (is_store);
  }
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (!is_store);
  }

  cp_imm_value: coverpoint get_rv32f_int_value({{20{instr.imm[11]}}, instr.imm}, 1);
  `ISACOV_CP_BITWISE_11_0(cp_imm_toggle, instr.imm, 1)

  cp_align_word: coverpoint instr.mem_addr {
    bins ALIGNED     = {0};
    bins UNALIGNED[] = {[1:3]};
  }
endgroup : cg_rv32f_ls

// c.flw and c.fsw (Zcf), with registers x8-x15 or f8-f15 and a 5-bit offset field
covergroup cg_rv32fc_ls(string name, bit is_store) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1 {
    bins RS1[] = {[8:15]};
  }
  cp_rd: coverpoint instr.rd {
    ignore_bins IGN_OFF = {[0:31]} with (is_store);
    bins RD[] = {[8:15]};
  }
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (!is_store);
    bins RS2[] = {[8:15]};
  }

  `ISACOV_CP_BITWISE_4_0(cp_imm_toggle, instr.imm[4:0], 1)

  cp_align_word: coverpoint instr.mem_addr {
    bins ALIGNED     = {0};
    bins UNALIGNED[] = {[1:3]};
  }
endgroup : cg_rv32fc_ls

// c.flwsp and c.fswsp (Zcf), sp-relative with a 6-bit offset field
covergroup cg_rv32fc_lssp(string name, bit is_store) with function sample(rv32f_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rd: coverpoint instr.rd {
    ignore_bins IGN_OFF = {[0:31]} with (is_store);
  }
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (!is_store);
  }

  `ISACOV_CP_BITWISE_5_0(cp_imm_toggle, instr.imm[5:0], 1)

  cp_align_word: coverpoint instr.mem_addr {
    bins ALIGNED     = {0};
    bins UNALIGNED[] = {[1:3]};
  }
endgroup : cg_rv32fc_lssp


class uvme_rv32f_isa_covg extends uvm_component;

  uvme_cv32e40p_cfg_c cfg;

  bit zfinx;

  uvm_analysis_imp#(uvma_isacov_mon_trn_c, uvme_rv32f_isa_covg) mon_trn_export;

  cg_rv32f_r            r_cg[rv32f_instr_name_t];
  cg_rv32f_r4           r4_cg[rv32f_instr_name_t];
  cg_rv32f_r1           r1_cg[rv32f_instr_name_t];
  cg_rv32f_cvt_to_int   cvt_to_int_cg[rv32f_instr_name_t];
  cg_rv32f_cvt_from_int cvt_from_int_cg[rv32f_instr_name_t];
  cg_rv32f_mv           mv_cg[rv32f_instr_name_t];
  cg_rv32f_cmp          cmp_cg[rv32f_instr_name_t];
  cg_rv32f_class        class_cg[rv32f_instr_name_t];
  cg_rv32f_ls           ls_cg[rv32f_instr_name_t];
  cg_rv32fc_ls          c_ls_cg[rv32f_instr_name_t];
  cg_rv32fc_lssp        c_lssp_cg[rv32f_instr_name_t];

  `uvm_component_utils_begin(uvme_rv32f_isa_covg)
    `uvm_field_object(cfg, UVM_DEFAULT)
  `uvm_component_utils_end

  extern function new(string name = "rv32f_isa_covg", uvm_component parent = null);
  extern function void build_phase(uvm_phase phase);
  extern function string cg_name(rv32f_instr_name_t name);
  extern function void write(uvma_isacov_mon_trn_c trn);
  extern function rv32f_instr_t get_instr(uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi, rv32f_instr_name_t name);

endclass : uvme_rv32f_isa_covg


function uvme_rv32f_isa_covg::new(string name = "rv32f_isa_covg", uvm_component parent = null);

  super.new(name, parent);

  mon_trn_export = new("mon_trn_export", this);

endfunction : new


function void uvme_rv32f_isa_covg::build_phase(uvm_phase phase);

  rv32f_instr_name_t r_rm[]  = '{FADD_S, FSUB_S, FMUL_S, FDIV_S};
  rv32f_instr_name_t r_nrm[] = '{FSGNJ_S, FSGNJN_S, FSGNJX_S, FMIN_S, FMAX_S};
  rv32f_instr_name_t r4[]    = '{FMADD_S, FMSUB_S, FNMSUB_S, FNMADD_S};
  rv32f_instr_name_t cmp[]   = '{FEQ_S, FLT_S, FLE_S};

  super.build_phase(phase);

  void'(uvm_config_db#(uvme_cv32e40p_cfg_c)::get(this, "", "cfg", cfg));
  if (cfg == null) begin
    `uvm_fatal("RV32FISACOVG", "Configuration handle is null")
  end

  zfinx = cfg.zfinx_fcov_en;

  // Zfinx keeps the FP operations on the integer registers and removes the loads,
  // stores and moves of the FP registers, Zcf included.
  foreach (r_rm[i])  r_cg[r_rm[i]]  = new(cg_name(r_rm[i]),  .has_rm(1), .values(zfinx));
  foreach (r_nrm[i]) r_cg[r_nrm[i]] = new(cg_name(r_nrm[i]), .has_rm(0), .values(zfinx));
  foreach (r4[i])    r4_cg[r4[i]]   = new(cg_name(r4[i]),    .values(zfinx));
  foreach (cmp[i])   cmp_cg[cmp[i]] = new(cg_name(cmp[i]),   .zfinx(zfinx));
  r1_cg[FSQRT_S]             = new(cg_name(FSQRT_S),   .values(zfinx));
  cvt_to_int_cg[FCVT_W_S]    = new(cg_name(FCVT_W_S),  .is_signed(1), .zfinx(zfinx));
  cvt_to_int_cg[FCVT_WU_S]   = new(cg_name(FCVT_WU_S), .is_signed(0), .zfinx(zfinx));
  cvt_from_int_cg[FCVT_S_W]  = new(cg_name(FCVT_S_W),  .is_signed(1), .zfinx(zfinx));
  cvt_from_int_cg[FCVT_S_WU] = new(cg_name(FCVT_S_WU), .is_signed(0), .zfinx(zfinx));
  class_cg[FCLASS_S] = new(cg_name(FCLASS_S), .zfinx(zfinx));

  if (!zfinx) begin
    mv_cg[FMV_X_W]     = new(cg_name(FMV_X_W), .to_int(1));
    mv_cg[FMV_W_X]     = new(cg_name(FMV_W_X), .to_int(0));
    ls_cg[FLW]         = new(cg_name(FLW),     .is_store(0));
    ls_cg[FSW]         = new(cg_name(FSW),     .is_store(1));
    c_ls_cg[C_FLW]     = new(cg_name(C_FLW),   .is_store(0));
    c_ls_cg[C_FSW]     = new(cg_name(C_FSW),   .is_store(1));
    c_lssp_cg[C_FLWSP] = new(cg_name(C_FLWSP), .is_store(0));
    c_lssp_cg[C_FSWSP] = new(cg_name(C_FSWSP), .is_store(1));
  end

endfunction : build_phase


function string uvme_rv32f_isa_covg::cg_name(rv32f_instr_name_t name);

  string ext = "rv32f";

  if (zfinx)
    ext = "rv32zfinx";
  else if (name inside {C_FLW, C_FSW, C_FLWSP, C_FSWSP})
    ext = "rv32fc";

  return $sformatf("%s_%s_cg", ext, name.name().tolower());

endfunction : cg_name


function void uvme_rv32f_isa_covg::write(uvma_isacov_mon_trn_c trn);

  uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi = trn.instr.rvfi;
  rv32f_instr_name_t                     name;
  rv32f_instr_t                          instr;

  // Same filter as uvma_isacov, which keeps the instructions retired without a trap and
  // the single-stepped ones without an exception.
  if (!((rvfi.trap[0] == 0) || ((rvfi.trap[11:9] == 4) && (rvfi.trap[1] == 0))))
    return;

  name = decode_rv32f(rvfi.insn);
  if (name == RV32F_UNKNOWN)
    return;

  instr = get_instr(rvfi, name);

  if (r_cg.exists(name))            r_cg[name].sample(instr);
  if (r4_cg.exists(name))           r4_cg[name].sample(instr);
  if (r1_cg.exists(name))           r1_cg[name].sample(instr);
  if (cvt_to_int_cg.exists(name))   cvt_to_int_cg[name].sample(instr);
  if (cvt_from_int_cg.exists(name)) cvt_from_int_cg[name].sample(instr);
  if (mv_cg.exists(name))           mv_cg[name].sample(instr);
  if (cmp_cg.exists(name))          cmp_cg[name].sample(instr);
  if (class_cg.exists(name))        class_cg[name].sample(instr);
  if (ls_cg.exists(name))           ls_cg[name].sample(instr);
  if (c_ls_cg.exists(name))         c_ls_cg[name].sample(instr);
  if (c_lssp_cg.exists(name))       c_lssp_cg[name].sample(instr);

endfunction : write


function rv32f_instr_t uvme_rv32f_isa_covg::get_instr(uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi, rv32f_instr_name_t name);

  bit [31:0]    insn = rvfi.insn;
  rv32f_instr_t instr;

  instr.name     = name;
  instr.rs1      = insn[19:15];
  instr.rs2      = insn[24:20];
  instr.rs3      = insn[31:27];
  instr.rd       = insn[11:7];
  instr.rm       = insn[14:12];
  instr.mem_addr = rvfi.mem_addr[1:0];

  case (name)
    FLW:     instr.imm = insn[31:20];
    FSW:     instr.imm = {insn[31:25], insn[11:7]};
    C_FLW,
    C_FSW: begin
      instr.rs1 = {2'b01, insn[9:7]};
      instr.rd  = {2'b01, insn[4:2]};
      instr.rs2 = {2'b01, insn[4:2]};
      instr.imm = {insn[5], insn[12:10], insn[6]};
    end
    C_FLWSP: begin
      instr.rs1 = 2;
      instr.imm = {insn[3:2], insn[12], insn[6:4]};
    end
    C_FSWSP: begin
      instr.rs1 = 2;
      instr.rs2 = insn[6:2];
      instr.imm = {insn[8:7], insn[12:9]};
    end
  endcase

  if (rvfi.csrs.exists("frm"))
    instr.frm = rvfi.csrs["frm"].rdata[2:0];

  // RVFI reports the integer registers only. They hold every operand with Zfinx, and the
  // GPR side of conversions, moves, compares and fclass with F.
  instr.rs1_value = rvfi.rs1_rdata;
  instr.rs2_value = rvfi.rs2_rdata;
  instr.rd_value  = rvfi.rd1_wdata;
  instr.rs1_class = get_rv32f_class(rvfi.rs1_rdata);
  instr.rs2_class = get_rv32f_class(rvfi.rs2_rdata);
  instr.rs3_class = get_rv32f_class(rvfi.rs3_rdata);
  instr.rd_class  = get_rv32f_class(rvfi.rd1_wdata);

  return instr;

endfunction : get_instr

`undef RV32F_CP_RM
`undef RV32F_CP_CLASS
`undef RV32F_CP_RD_CLASS
`undef RV32F_CP_HAZARD

`endif // __UVME_RV32F_ISA_COVG_SV__
