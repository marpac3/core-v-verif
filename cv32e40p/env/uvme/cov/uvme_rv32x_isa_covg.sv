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

// Xpulp instruction coverage (XPULPV2, and cv.elw with COREV_CLUSTER). Like
// uvme_rv32f_isa_covg, it decodes the instructions of the uvma_isacov transactions from
// rvfi.insn (INSTR_CV_* encodings) and samples one covergroup per instruction. Register
// values are taken raw, so signed and unsigned variants share the same bins. The hardware
// loop CSRs and events are covered by uvme_rv32x_hwloop_covg.
//
// The RVFI ports carry what the cv32e40p decoder puts on them.
// - rs2_rdata is the register of the rs2 field, meaningless for the immediate forms.
// - rs3_rdata is port c, which reads the rd field. That is a third source for mac, macN,
//   sdot, insert, addNr, shuffle2, packhi/lo and cplxmul, and the offset register of the
//   reg-reg stores.
// - rd1 is the EX write, the rs1 update of a post-increment access.
// - rd2 is the WB write, the data of a post-increment load.

`ifndef __UVME_RV32X_ISA_COVG_SV__
`define __UVME_RV32X_ISA_COVG_SV__

typedef enum {
  RV32X_UNKNOWN,
  CV_LB_PI_RI, CV_LH_PI_RI, CV_LW_PI_RI, CV_ELW, CV_LBU_PI_RI, CV_LHU_PI_RI,
  CV_BEQIMM, CV_BNEIMM, CV_LB_PI_RR, CV_LH_PI_RR, CV_LW_PI_RR, CV_LBU_PI_RR,
  CV_LHU_PI_RR, CV_LB_RR, CV_LH_RR, CV_LW_RR, CV_LBU_RR, CV_LHU_RR,
  CV_SB_PI_RI, CV_SH_PI_RI, CV_SW_PI_RI, CV_SB_PI_RR, CV_SH_PI_RR, CV_SW_PI_RR,
  CV_SB_RR, CV_SH_RR, CV_SW_RR, CV_STARTI_0, CV_START_0, CV_ENDI_0,
  CV_END_0, CV_COUNTI_0, CV_COUNT_0, CV_SETUPI_0, CV_SETUP_0, CV_STARTI_1,
  CV_START_1, CV_ENDI_1, CV_END_1, CV_COUNTI_1, CV_COUNT_1, CV_SETUPI_1,
  CV_SETUP_1, CV_EXTRACTR, CV_EXTRACTUR, CV_INSERTR, CV_BCLRR, CV_BSETR,
  CV_ROR, CV_FF1, CV_FL1, CV_CLB, CV_CNT, CV_ABS,
  CV_SLE, CV_SLEU, CV_MIN, CV_MINU, CV_MAX, CV_MAXU,
  CV_EXTHS, CV_EXTHZ, CV_EXTBS, CV_EXTBZ, CV_CLIP, CV_CLIPU,
  CV_CLIPR, CV_CLIPUR, CV_ADDNR, CV_ADDUNR, CV_ADDRNR, CV_ADDURNR,
  CV_SUBNR, CV_SUBUNR, CV_SUBRNR, CV_SUBURNR, CV_MAC, CV_MSU,
  CV_EXTRACT, CV_EXTRACTU, CV_INSERT, CV_BCLR, CV_BSET, CV_BITREV,
  CV_ADDN, CV_ADDUN, CV_ADDRN, CV_ADDURN, CV_SUBN, CV_SUBUN,
  CV_SUBRN, CV_SUBURN, CV_MULSN, CV_MULHHSN, CV_MULSRN, CV_MULHHSRN,
  CV_MULUN, CV_MULHHUN, CV_MULURN, CV_MULHHURN, CV_MACSN, CV_MACHHSN,
  CV_MACSRN, CV_MACHHSRN, CV_MACUN, CV_MACHHUN, CV_MACURN, CV_MACHHURN,
  CV_ADD_H, CV_ADD_SC_H, CV_ADD_SCI_H, CV_ADD_B, CV_ADD_SC_B, CV_ADD_SCI_B,
  CV_SUB_H, CV_SUB_SC_H, CV_SUB_SCI_H, CV_SUB_B, CV_SUB_SC_B, CV_SUB_SCI_B,
  CV_AVG_H, CV_AVG_SC_H, CV_AVG_SCI_H, CV_AVG_B, CV_AVG_SC_B, CV_AVG_SCI_B,
  CV_AVGU_H, CV_AVGU_SC_H, CV_AVGU_SCI_H, CV_AVGU_B, CV_AVGU_SC_B, CV_AVGU_SCI_B,
  CV_MIN_H, CV_MIN_SC_H, CV_MIN_SCI_H, CV_MIN_B, CV_MIN_SC_B, CV_MIN_SCI_B,
  CV_MINU_H, CV_MINU_SC_H, CV_MINU_SCI_H, CV_MINU_B, CV_MINU_SC_B, CV_MINU_SCI_B,
  CV_MAX_H, CV_MAX_SC_H, CV_MAX_SCI_H, CV_MAX_B, CV_MAX_SC_B, CV_MAX_SCI_B,
  CV_MAXU_H, CV_MAXU_SC_H, CV_MAXU_SCI_H, CV_MAXU_B, CV_MAXU_SC_B, CV_MAXU_SCI_B,
  CV_SRL_H, CV_SRL_SC_H, CV_SRL_SCI_H, CV_SRL_B, CV_SRL_SC_B, CV_SRL_SCI_B,
  CV_SRA_H, CV_SRA_SC_H, CV_SRA_SCI_H, CV_SRA_B, CV_SRA_SC_B, CV_SRA_SCI_B,
  CV_SLL_H, CV_SLL_SC_H, CV_SLL_SCI_H, CV_SLL_B, CV_SLL_SC_B, CV_SLL_SCI_B,
  CV_OR_H, CV_OR_SC_H, CV_OR_SCI_H, CV_OR_B, CV_OR_SC_B, CV_OR_SCI_B,
  CV_XOR_H, CV_XOR_SC_H, CV_XOR_SCI_H, CV_XOR_B, CV_XOR_SC_B, CV_XOR_SCI_B,
  CV_AND_H, CV_AND_SC_H, CV_AND_SCI_H, CV_AND_B, CV_AND_SC_B, CV_AND_SCI_B,
  CV_ABS_H, CV_ABS_B, CV_DOTUP_H, CV_DOTUP_SC_H, CV_DOTUP_SCI_H, CV_DOTUP_B,
  CV_DOTUP_SC_B, CV_DOTUP_SCI_B, CV_DOTUSP_H, CV_DOTUSP_SC_H, CV_DOTUSP_SCI_H, CV_DOTUSP_B,
  CV_DOTUSP_SC_B, CV_DOTUSP_SCI_B, CV_DOTSP_H, CV_DOTSP_SC_H, CV_DOTSP_SCI_H, CV_DOTSP_B,
  CV_DOTSP_SC_B, CV_DOTSP_SCI_B, CV_SDOTUP_H, CV_SDOTUP_SC_H, CV_SDOTUP_SCI_H, CV_SDOTUP_B,
  CV_SDOTUP_SC_B, CV_SDOTUP_SCI_B, CV_SDOTUSP_H, CV_SDOTUSP_SC_H, CV_SDOTUSP_SCI_H, CV_SDOTUSP_B,
  CV_SDOTUSP_SC_B, CV_SDOTUSP_SCI_B, CV_SDOTSP_H, CV_SDOTSP_SC_H, CV_SDOTSP_SCI_H, CV_SDOTSP_B,
  CV_SDOTSP_SC_B, CV_SDOTSP_SCI_B, CV_EXTRACT_H, CV_EXTRACT_B, CV_EXTRACTU_H, CV_EXTRACTU_B,
  CV_INSERT_H, CV_INSERT_B, CV_SHUFFLE_H, CV_SHUFFLE_SCI_H, CV_SHUFFLE_B, CV_SHUFFLEI0_SCI_B,
  CV_SHUFFLEI1_SCI_B, CV_SHUFFLEI2_SCI_B, CV_SHUFFLEI3_SCI_B, CV_SHUFFLE2_H, CV_SHUFFLE2_B, CV_PACK,
  CV_PACK_H, CV_PACKHI_B, CV_PACKLO_B, CV_CMPEQ_H, CV_CMPEQ_SC_H, CV_CMPEQ_SCI_H,
  CV_CMPEQ_B, CV_CMPEQ_SC_B, CV_CMPEQ_SCI_B, CV_CMPNE_H, CV_CMPNE_SC_H, CV_CMPNE_SCI_H,
  CV_CMPNE_B, CV_CMPNE_SC_B, CV_CMPNE_SCI_B, CV_CMPGT_H, CV_CMPGT_SC_H, CV_CMPGT_SCI_H,
  CV_CMPGT_B, CV_CMPGT_SC_B, CV_CMPGT_SCI_B, CV_CMPGE_H, CV_CMPGE_SC_H, CV_CMPGE_SCI_H,
  CV_CMPGE_B, CV_CMPGE_SC_B, CV_CMPGE_SCI_B, CV_CMPLT_H, CV_CMPLT_SC_H, CV_CMPLT_SCI_H,
  CV_CMPLT_B, CV_CMPLT_SC_B, CV_CMPLT_SCI_B, CV_CMPLE_H, CV_CMPLE_SC_H, CV_CMPLE_SCI_H,
  CV_CMPLE_B, CV_CMPLE_SC_B, CV_CMPLE_SCI_B, CV_CMPGTU_H, CV_CMPGTU_SC_H, CV_CMPGTU_SCI_H,
  CV_CMPGTU_B, CV_CMPGTU_SC_B, CV_CMPGTU_SCI_B, CV_CMPGEU_H, CV_CMPGEU_SC_H, CV_CMPGEU_SCI_H,
  CV_CMPGEU_B, CV_CMPGEU_SC_B, CV_CMPGEU_SCI_B, CV_CMPLTU_H, CV_CMPLTU_SC_H, CV_CMPLTU_SCI_H,
  CV_CMPLTU_B, CV_CMPLTU_SC_B, CV_CMPLTU_SCI_B, CV_CMPLEU_H, CV_CMPLEU_SC_H, CV_CMPLEU_SCI_H,
  CV_CMPLEU_B, CV_CMPLEU_SC_B, CV_CMPLEU_SCI_B, CV_CPLXMUL_R, CV_CPLXMUL_R_DIV2, CV_CPLXMUL_R_DIV4,
  CV_CPLXMUL_R_DIV8, CV_CPLXMUL_I, CV_CPLXMUL_I_DIV2, CV_CPLXMUL_I_DIV4, CV_CPLXMUL_I_DIV8, CV_CPLXCONJ,
  CV_SUBROTMJ, CV_SUBROTMJ_DIV2, CV_SUBROTMJ_DIV4, CV_SUBROTMJ_DIV8, CV_ADD_DIV2, CV_ADD_DIV4,
  CV_ADD_DIV8, CV_SUB_DIV2, CV_SUB_DIV4, CV_SUB_DIV8
} rv32x_instr_name_t;

typedef enum {
  RV32X_ZERO,
  RV32X_POSITIVE,
  RV32X_NEGATIVE
} rv32x_sign_t;

// Kind of SIMD destination: one value per lane, a mask per lane for the compares, or one
// scalar for the dot products and extract
typedef enum {
  RV32X_RD_LANES,
  RV32X_RD_MASK,
  RV32X_RD_SCALAR
} rv32x_rd_kind_t;

// How rd is read as a third source: not at all, as lanes, or as a scalar (sdot accumulators)
typedef enum {
  RV32X_RS3_NONE,
  RV32X_RS3_LANES,
  RV32X_RS3_SCALAR
} rv32x_rs3_kind_t;

// Imm6 of the SIMD operations: none, sign-extended, zero-extended, or a field of which
// only the low bits are used (shift amount, lane index, shuffle selector)
typedef enum {
  RV32X_IMM6_NONE,
  RV32X_IMM6_SIGNED,
  RV32X_IMM6_UNSIGNED,
  RV32X_IMM6_FIELD
} rv32x_imm6_kind_t;

typedef struct {
  rv32x_instr_name_t name;
  bit [4:0]          rs1;
  bit [4:0]          rs2;
  bit [4:0]          rd;
  bit [4:0]          rs3;       // rd field as offset register of the reg-reg stores
  bit [11:0]         imm12;     // offset of loads and stores, or uimmL of the hardware loops
  bit [5:0]          imm6;      // Imm6 of the SIMD operations
  bit [4:0]          is2;       // Is2, Imm5 of the immediate branches or uimmS of cv.setupi
  bit [4:0]          is3;
  bit [1:0]          mem_addr;  // low bits of the data address
  bit                taken;
  bit [31:0]         rs1_value;
  bit [31:0]         rs2_value;
  bit [31:0]         rs3_value;
  bit [31:0]         rd_value;
} rv32x_instr_t;

function automatic rv32x_sign_t get_rv32x_sign(bit [31:0] value);
  if (value == 0)
    return RV32X_ZERO;
  return value[31] ? RV32X_NEGATIVE : RV32X_POSITIVE;
endfunction : get_rv32x_sign

// Sign of one lane, with lanes 2 for halfwords or 4 for bytes
function automatic rv32x_sign_t get_rv32x_lane_sign(bit [31:0] value, int unsigned lanes, int unsigned lane);
  bit [15:0] h;
  bit [7:0]  b;

  if (lane >= lanes)
    return RV32X_ZERO;
  if (lanes == 2) begin
    h = value[16*lane +: 16];
    return (h == 0) ? RV32X_ZERO : h[15] ? RV32X_NEGATIVE : RV32X_POSITIVE;
  end
  b = value[8*lane +: 8];
  return (b == 0) ? RV32X_ZERO : b[7] ? RV32X_NEGATIVE : RV32X_POSITIVE;
endfunction : get_rv32x_lane_sign

function automatic rv32x_instr_name_t decode_rv32x(bit [31:0] insn);
  casez (insn)
    INSTR_CV_LB_PI_RI:         return CV_LB_PI_RI;
    INSTR_CV_LH_PI_RI:         return CV_LH_PI_RI;
    INSTR_CV_LW_PI_RI:         return CV_LW_PI_RI;
    INSTR_CV_ELW_PI_RI:        return CV_ELW;
    INSTR_CV_LBU_PI_RI:        return CV_LBU_PI_RI;
    INSTR_CV_LHU_PI_RI:        return CV_LHU_PI_RI;
    INSTR_CV_BEQIMM:           return CV_BEQIMM;
    INSTR_CV_BNEIMM:           return CV_BNEIMM;
    INSTR_CV_LB_PI_RR:         return CV_LB_PI_RR;
    INSTR_CV_LH_PI_RR:         return CV_LH_PI_RR;
    INSTR_CV_LW_PI_RR:         return CV_LW_PI_RR;
    INSTR_CV_LBU_PI_RR:        return CV_LBU_PI_RR;
    INSTR_CV_LHU_PI_RR:        return CV_LHU_PI_RR;
    INSTR_CV_LB_RR:            return CV_LB_RR;
    INSTR_CV_LH_RR:            return CV_LH_RR;
    INSTR_CV_LW_RR:            return CV_LW_RR;
    INSTR_CV_LBU_RR:           return CV_LBU_RR;
    INSTR_CV_LHU_RR:           return CV_LHU_RR;
    INSTR_CV_SB_PI_RI:         return CV_SB_PI_RI;
    INSTR_CV_SH_PI_RI:         return CV_SH_PI_RI;
    INSTR_CV_SW_PI_RI:         return CV_SW_PI_RI;
    INSTR_CV_SB_PI_RR:         return CV_SB_PI_RR;
    INSTR_CV_SH_PI_RR:         return CV_SH_PI_RR;
    INSTR_CV_SW_PI_RR:         return CV_SW_PI_RR;
    INSTR_CV_SB_RR:            return CV_SB_RR;
    INSTR_CV_SH_RR:            return CV_SH_RR;
    INSTR_CV_SW_RR:            return CV_SW_RR;
    INSTR_CV_STARTI_0:         return CV_STARTI_0;
    INSTR_CV_START_0:          return CV_START_0;
    INSTR_CV_ENDI_0:           return CV_ENDI_0;
    INSTR_CV_END_0:            return CV_END_0;
    INSTR_CV_COUNTI_0:         return CV_COUNTI_0;
    INSTR_CV_COUNT_0:          return CV_COUNT_0;
    INSTR_CV_SETUPI_0:         return CV_SETUPI_0;
    INSTR_CV_SETUP_0:          return CV_SETUP_0;
    INSTR_CV_STARTI_1:         return CV_STARTI_1;
    INSTR_CV_START_1:          return CV_START_1;
    INSTR_CV_ENDI_1:           return CV_ENDI_1;
    INSTR_CV_END_1:            return CV_END_1;
    INSTR_CV_COUNTI_1:         return CV_COUNTI_1;
    INSTR_CV_COUNT_1:          return CV_COUNT_1;
    INSTR_CV_SETUPI_1:         return CV_SETUPI_1;
    INSTR_CV_SETUP_1:          return CV_SETUP_1;
    INSTR_CV_EXTRACTR:         return CV_EXTRACTR;
    INSTR_CV_EXTRACTUR:        return CV_EXTRACTUR;
    INSTR_CV_INSERTR:          return CV_INSERTR;
    INSTR_CV_BCLRR:            return CV_BCLRR;
    INSTR_CV_BSETR:            return CV_BSETR;
    INSTR_CV_ROR:              return CV_ROR;
    INSTR_CV_FF1:              return CV_FF1;
    INSTR_CV_FL1:              return CV_FL1;
    INSTR_CV_CLB:              return CV_CLB;
    INSTR_CV_CNT:              return CV_CNT;
    INSTR_CV_ABS:              return CV_ABS;
    INSTR_CV_SLE:              return CV_SLE;
    INSTR_CV_SLEU:             return CV_SLEU;
    INSTR_CV_MIN:              return CV_MIN;
    INSTR_CV_MINU:             return CV_MINU;
    INSTR_CV_MAX:              return CV_MAX;
    INSTR_CV_MAXU:             return CV_MAXU;
    INSTR_CV_EXTHS:            return CV_EXTHS;
    INSTR_CV_EXTHZ:            return CV_EXTHZ;
    INSTR_CV_EXTBS:            return CV_EXTBS;
    INSTR_CV_EXTBZ:            return CV_EXTBZ;
    INSTR_CV_CLIP:             return CV_CLIP;
    INSTR_CV_CLIPU:            return CV_CLIPU;
    INSTR_CV_CLIPR:            return CV_CLIPR;
    INSTR_CV_CLIPUR:           return CV_CLIPUR;
    INSTR_CV_ADDNR:            return CV_ADDNR;
    INSTR_CV_ADDUNR:           return CV_ADDUNR;
    INSTR_CV_ADDRNR:           return CV_ADDRNR;
    INSTR_CV_ADDURNR:          return CV_ADDURNR;
    INSTR_CV_SUBNR:            return CV_SUBNR;
    INSTR_CV_SUBUNR:           return CV_SUBUNR;
    INSTR_CV_SUBRNR:           return CV_SUBRNR;
    INSTR_CV_SUBURNR:          return CV_SUBURNR;
    INSTR_CV_MAC:              return CV_MAC;
    INSTR_CV_MSU:              return CV_MSU;
    INSTR_CV_EXTRACT:          return CV_EXTRACT;
    INSTR_CV_EXTRACTU:         return CV_EXTRACTU;
    INSTR_CV_INSERT:           return CV_INSERT;
    INSTR_CV_BCLR:             return CV_BCLR;
    INSTR_CV_BSET:             return CV_BSET;
    INSTR_CV_BITREV:           return CV_BITREV;
    INSTR_CV_ADDN:             return CV_ADDN;
    INSTR_CV_ADDUN:            return CV_ADDUN;
    INSTR_CV_ADDRN:            return CV_ADDRN;
    INSTR_CV_ADDURN:           return CV_ADDURN;
    INSTR_CV_SUBN:             return CV_SUBN;
    INSTR_CV_SUBUN:            return CV_SUBUN;
    INSTR_CV_SUBRN:            return CV_SUBRN;
    INSTR_CV_SUBURN:           return CV_SUBURN;
    INSTR_CV_MULSN:            return CV_MULSN;
    INSTR_CV_MULHHSN:          return CV_MULHHSN;
    INSTR_CV_MULSRN:           return CV_MULSRN;
    INSTR_CV_MULHHSRN:         return CV_MULHHSRN;
    INSTR_CV_MULUN:            return CV_MULUN;
    INSTR_CV_MULHHUN:          return CV_MULHHUN;
    INSTR_CV_MULURN:           return CV_MULURN;
    INSTR_CV_MULHHURN:         return CV_MULHHURN;
    INSTR_CV_MACSN:            return CV_MACSN;
    INSTR_CV_MACHHSN:          return CV_MACHHSN;
    INSTR_CV_MACSRN:           return CV_MACSRN;
    INSTR_CV_MACHHSRN:         return CV_MACHHSRN;
    INSTR_CV_MACUN:            return CV_MACUN;
    INSTR_CV_MACHHUN:          return CV_MACHHUN;
    INSTR_CV_MACURN:           return CV_MACURN;
    INSTR_CV_MACHHURN:         return CV_MACHHURN;
    INSTR_CV_ADD_H:            return CV_ADD_H;
    INSTR_CV_ADD_SC_H:         return CV_ADD_SC_H;
    INSTR_CV_ADD_SCI_H:        return CV_ADD_SCI_H;
    INSTR_CV_ADD_B:            return CV_ADD_B;
    INSTR_CV_ADD_SC_B:         return CV_ADD_SC_B;
    INSTR_CV_ADD_SCI_B:        return CV_ADD_SCI_B;
    INSTR_CV_SUB_H:            return CV_SUB_H;
    INSTR_CV_SUB_SC_H:         return CV_SUB_SC_H;
    INSTR_CV_SUB_SCI_H:        return CV_SUB_SCI_H;
    INSTR_CV_SUB_B:            return CV_SUB_B;
    INSTR_CV_SUB_SC_B:         return CV_SUB_SC_B;
    INSTR_CV_SUB_SCI_B:        return CV_SUB_SCI_B;
    INSTR_CV_AVG_H:            return CV_AVG_H;
    INSTR_CV_AVG_SC_H:         return CV_AVG_SC_H;
    INSTR_CV_AVG_SCI_H:        return CV_AVG_SCI_H;
    INSTR_CV_AVG_B:            return CV_AVG_B;
    INSTR_CV_AVG_SC_B:         return CV_AVG_SC_B;
    INSTR_CV_AVG_SCI_B:        return CV_AVG_SCI_B;
    INSTR_CV_AVGU_H:           return CV_AVGU_H;
    INSTR_CV_AVGU_SC_H:        return CV_AVGU_SC_H;
    INSTR_CV_AVGU_SCI_H:       return CV_AVGU_SCI_H;
    INSTR_CV_AVGU_B:           return CV_AVGU_B;
    INSTR_CV_AVGU_SC_B:        return CV_AVGU_SC_B;
    INSTR_CV_AVGU_SCI_B:       return CV_AVGU_SCI_B;
    INSTR_CV_MIN_H:            return CV_MIN_H;
    INSTR_CV_MIN_SC_H:         return CV_MIN_SC_H;
    INSTR_CV_MIN_SCI_H:        return CV_MIN_SCI_H;
    INSTR_CV_MIN_B:            return CV_MIN_B;
    INSTR_CV_MIN_SC_B:         return CV_MIN_SC_B;
    INSTR_CV_MIN_SCI_B:        return CV_MIN_SCI_B;
    INSTR_CV_MINU_H:           return CV_MINU_H;
    INSTR_CV_MINU_SC_H:        return CV_MINU_SC_H;
    INSTR_CV_MINU_SCI_H:       return CV_MINU_SCI_H;
    INSTR_CV_MINU_B:           return CV_MINU_B;
    INSTR_CV_MINU_SC_B:        return CV_MINU_SC_B;
    INSTR_CV_MINU_SCI_B:       return CV_MINU_SCI_B;
    INSTR_CV_MAX_H:            return CV_MAX_H;
    INSTR_CV_MAX_SC_H:         return CV_MAX_SC_H;
    INSTR_CV_MAX_SCI_H:        return CV_MAX_SCI_H;
    INSTR_CV_MAX_B:            return CV_MAX_B;
    INSTR_CV_MAX_SC_B:         return CV_MAX_SC_B;
    INSTR_CV_MAX_SCI_B:        return CV_MAX_SCI_B;
    INSTR_CV_MAXU_H:           return CV_MAXU_H;
    INSTR_CV_MAXU_SC_H:        return CV_MAXU_SC_H;
    INSTR_CV_MAXU_SCI_H:       return CV_MAXU_SCI_H;
    INSTR_CV_MAXU_B:           return CV_MAXU_B;
    INSTR_CV_MAXU_SC_B:        return CV_MAXU_SC_B;
    INSTR_CV_MAXU_SCI_B:       return CV_MAXU_SCI_B;
    INSTR_CV_SRL_H:            return CV_SRL_H;
    INSTR_CV_SRL_SC_H:         return CV_SRL_SC_H;
    INSTR_CV_SRL_SCI_H:        return CV_SRL_SCI_H;
    INSTR_CV_SRL_B:            return CV_SRL_B;
    INSTR_CV_SRL_SC_B:         return CV_SRL_SC_B;
    INSTR_CV_SRL_SCI_B:        return CV_SRL_SCI_B;
    INSTR_CV_SRA_H:            return CV_SRA_H;
    INSTR_CV_SRA_SC_H:         return CV_SRA_SC_H;
    INSTR_CV_SRA_SCI_H:        return CV_SRA_SCI_H;
    INSTR_CV_SRA_B:            return CV_SRA_B;
    INSTR_CV_SRA_SC_B:         return CV_SRA_SC_B;
    INSTR_CV_SRA_SCI_B:        return CV_SRA_SCI_B;
    INSTR_CV_SLL_H:            return CV_SLL_H;
    INSTR_CV_SLL_SC_H:         return CV_SLL_SC_H;
    INSTR_CV_SLL_SCI_H:        return CV_SLL_SCI_H;
    INSTR_CV_SLL_B:            return CV_SLL_B;
    INSTR_CV_SLL_SC_B:         return CV_SLL_SC_B;
    INSTR_CV_SLL_SCI_B:        return CV_SLL_SCI_B;
    INSTR_CV_OR_H:             return CV_OR_H;
    INSTR_CV_OR_SC_H:          return CV_OR_SC_H;
    INSTR_CV_OR_SCI_H:         return CV_OR_SCI_H;
    INSTR_CV_OR_B:             return CV_OR_B;
    INSTR_CV_OR_SC_B:          return CV_OR_SC_B;
    INSTR_CV_OR_SCI_B:         return CV_OR_SCI_B;
    INSTR_CV_XOR_H:            return CV_XOR_H;
    INSTR_CV_XOR_SC_H:         return CV_XOR_SC_H;
    INSTR_CV_XOR_SCI_H:        return CV_XOR_SCI_H;
    INSTR_CV_XOR_B:            return CV_XOR_B;
    INSTR_CV_XOR_SC_B:         return CV_XOR_SC_B;
    INSTR_CV_XOR_SCI_B:        return CV_XOR_SCI_B;
    INSTR_CV_AND_H:            return CV_AND_H;
    INSTR_CV_AND_SC_H:         return CV_AND_SC_H;
    INSTR_CV_AND_SCI_H:        return CV_AND_SCI_H;
    INSTR_CV_AND_B:            return CV_AND_B;
    INSTR_CV_AND_SC_B:         return CV_AND_SC_B;
    INSTR_CV_AND_SCI_B:        return CV_AND_SCI_B;
    INSTR_CV_ABS_H:            return CV_ABS_H;
    INSTR_CV_ABS_B:            return CV_ABS_B;
    INSTR_CV_DOTUP_H:          return CV_DOTUP_H;
    INSTR_CV_DOTUP_SC_H:       return CV_DOTUP_SC_H;
    INSTR_CV_DOTUP_SCI_H:      return CV_DOTUP_SCI_H;
    INSTR_CV_DOTUP_B:          return CV_DOTUP_B;
    INSTR_CV_DOTUP_SC_B:       return CV_DOTUP_SC_B;
    INSTR_CV_DOTUP_SCI_B:      return CV_DOTUP_SCI_B;
    INSTR_CV_DOTUSP_H:         return CV_DOTUSP_H;
    INSTR_CV_DOTUSP_SC_H:      return CV_DOTUSP_SC_H;
    INSTR_CV_DOTUSP_SCI_H:     return CV_DOTUSP_SCI_H;
    INSTR_CV_DOTUSP_B:         return CV_DOTUSP_B;
    INSTR_CV_DOTUSP_SC_B:      return CV_DOTUSP_SC_B;
    INSTR_CV_DOTUSP_SCI_B:     return CV_DOTUSP_SCI_B;
    INSTR_CV_DOTSP_H:          return CV_DOTSP_H;
    INSTR_CV_DOTSP_SC_H:       return CV_DOTSP_SC_H;
    INSTR_CV_DOTSP_SCI_H:      return CV_DOTSP_SCI_H;
    INSTR_CV_DOTSP_B:          return CV_DOTSP_B;
    INSTR_CV_DOTSP_SC_B:       return CV_DOTSP_SC_B;
    INSTR_CV_DOTSP_SCI_B:      return CV_DOTSP_SCI_B;
    INSTR_CV_SDOTUP_H:         return CV_SDOTUP_H;
    INSTR_CV_SDOTUP_SC_H:      return CV_SDOTUP_SC_H;
    INSTR_CV_SDOTUP_SCI_H:     return CV_SDOTUP_SCI_H;
    INSTR_CV_SDOTUP_B:         return CV_SDOTUP_B;
    INSTR_CV_SDOTUP_SC_B:      return CV_SDOTUP_SC_B;
    INSTR_CV_SDOTUP_SCI_B:     return CV_SDOTUP_SCI_B;
    INSTR_CV_SDOTUSP_H:        return CV_SDOTUSP_H;
    INSTR_CV_SDOTUSP_SC_H:     return CV_SDOTUSP_SC_H;
    INSTR_CV_SDOTUSP_SCI_H:    return CV_SDOTUSP_SCI_H;
    INSTR_CV_SDOTUSP_B:        return CV_SDOTUSP_B;
    INSTR_CV_SDOTUSP_SC_B:     return CV_SDOTUSP_SC_B;
    INSTR_CV_SDOTUSP_SCI_B:    return CV_SDOTUSP_SCI_B;
    INSTR_CV_SDOTSP_H:         return CV_SDOTSP_H;
    INSTR_CV_SDOTSP_SC_H:      return CV_SDOTSP_SC_H;
    INSTR_CV_SDOTSP_SCI_H:     return CV_SDOTSP_SCI_H;
    INSTR_CV_SDOTSP_B:         return CV_SDOTSP_B;
    INSTR_CV_SDOTSP_SC_B:      return CV_SDOTSP_SC_B;
    INSTR_CV_SDOTSP_SCI_B:     return CV_SDOTSP_SCI_B;
    INSTR_CV_EXTRACT_H:        return CV_EXTRACT_H;
    INSTR_CV_EXTRACT_B:        return CV_EXTRACT_B;
    INSTR_CV_EXTRACTU_H:       return CV_EXTRACTU_H;
    INSTR_CV_EXTRACTU_B:       return CV_EXTRACTU_B;
    INSTR_CV_INSERT_H:         return CV_INSERT_H;
    INSTR_CV_INSERT_B:         return CV_INSERT_B;
    INSTR_CV_SHUFFLE_H:        return CV_SHUFFLE_H;
    INSTR_CV_SHUFFLE_SCI_H:    return CV_SHUFFLE_SCI_H;
    INSTR_CV_SHUFFLE_B:        return CV_SHUFFLE_B;
    INSTR_CV_SHUFFLEI0_SCI_B:  return CV_SHUFFLEI0_SCI_B;
    INSTR_CV_SHUFFLEI1_SCI_B:  return CV_SHUFFLEI1_SCI_B;
    INSTR_CV_SHUFFLEI2_SCI_B:  return CV_SHUFFLEI2_SCI_B;
    INSTR_CV_SHUFFLEI3_SCI_B:  return CV_SHUFFLEI3_SCI_B;
    INSTR_CV_SHUFFLE2_H:       return CV_SHUFFLE2_H;
    INSTR_CV_SHUFFLE2_B:       return CV_SHUFFLE2_B;
    INSTR_CV_PACK:             return CV_PACK;
    INSTR_CV_PACK_H:           return CV_PACK_H;
    INSTR_CV_PACKHI_B:         return CV_PACKHI_B;
    INSTR_CV_PACKLO_B:         return CV_PACKLO_B;
    INSTR_CV_CMPEQ_H:          return CV_CMPEQ_H;
    INSTR_CV_CMPEQ_SC_H:       return CV_CMPEQ_SC_H;
    INSTR_CV_CMPEQ_SCI_H:      return CV_CMPEQ_SCI_H;
    INSTR_CV_CMPEQ_B:          return CV_CMPEQ_B;
    INSTR_CV_CMPEQ_SC_B:       return CV_CMPEQ_SC_B;
    INSTR_CV_CMPEQ_SCI_B:      return CV_CMPEQ_SCI_B;
    INSTR_CV_CMPNE_H:          return CV_CMPNE_H;
    INSTR_CV_CMPNE_SC_H:       return CV_CMPNE_SC_H;
    INSTR_CV_CMPNE_SCI_H:      return CV_CMPNE_SCI_H;
    INSTR_CV_CMPNE_B:          return CV_CMPNE_B;
    INSTR_CV_CMPNE_SC_B:       return CV_CMPNE_SC_B;
    INSTR_CV_CMPNE_SCI_B:      return CV_CMPNE_SCI_B;
    INSTR_CV_CMPGT_H:          return CV_CMPGT_H;
    INSTR_CV_CMPGT_SC_H:       return CV_CMPGT_SC_H;
    INSTR_CV_CMPGT_SCI_H:      return CV_CMPGT_SCI_H;
    INSTR_CV_CMPGT_B:          return CV_CMPGT_B;
    INSTR_CV_CMPGT_SC_B:       return CV_CMPGT_SC_B;
    INSTR_CV_CMPGT_SCI_B:      return CV_CMPGT_SCI_B;
    INSTR_CV_CMPGE_H:          return CV_CMPGE_H;
    INSTR_CV_CMPGE_SC_H:       return CV_CMPGE_SC_H;
    INSTR_CV_CMPGE_SCI_H:      return CV_CMPGE_SCI_H;
    INSTR_CV_CMPGE_B:          return CV_CMPGE_B;
    INSTR_CV_CMPGE_SC_B:       return CV_CMPGE_SC_B;
    INSTR_CV_CMPGE_SCI_B:      return CV_CMPGE_SCI_B;
    INSTR_CV_CMPLT_H:          return CV_CMPLT_H;
    INSTR_CV_CMPLT_SC_H:       return CV_CMPLT_SC_H;
    INSTR_CV_CMPLT_SCI_H:      return CV_CMPLT_SCI_H;
    INSTR_CV_CMPLT_B:          return CV_CMPLT_B;
    INSTR_CV_CMPLT_SC_B:       return CV_CMPLT_SC_B;
    INSTR_CV_CMPLT_SCI_B:      return CV_CMPLT_SCI_B;
    INSTR_CV_CMPLE_H:          return CV_CMPLE_H;
    INSTR_CV_CMPLE_SC_H:       return CV_CMPLE_SC_H;
    INSTR_CV_CMPLE_SCI_H:      return CV_CMPLE_SCI_H;
    INSTR_CV_CMPLE_B:          return CV_CMPLE_B;
    INSTR_CV_CMPLE_SC_B:       return CV_CMPLE_SC_B;
    INSTR_CV_CMPLE_SCI_B:      return CV_CMPLE_SCI_B;
    INSTR_CV_CMPGTU_H:         return CV_CMPGTU_H;
    INSTR_CV_CMPGTU_SC_H:      return CV_CMPGTU_SC_H;
    INSTR_CV_CMPGTU_SCI_H:     return CV_CMPGTU_SCI_H;
    INSTR_CV_CMPGTU_B:         return CV_CMPGTU_B;
    INSTR_CV_CMPGTU_SC_B:      return CV_CMPGTU_SC_B;
    INSTR_CV_CMPGTU_SCI_B:     return CV_CMPGTU_SCI_B;
    INSTR_CV_CMPGEU_H:         return CV_CMPGEU_H;
    INSTR_CV_CMPGEU_SC_H:      return CV_CMPGEU_SC_H;
    INSTR_CV_CMPGEU_SCI_H:     return CV_CMPGEU_SCI_H;
    INSTR_CV_CMPGEU_B:         return CV_CMPGEU_B;
    INSTR_CV_CMPGEU_SC_B:      return CV_CMPGEU_SC_B;
    INSTR_CV_CMPGEU_SCI_B:     return CV_CMPGEU_SCI_B;
    INSTR_CV_CMPLTU_H:         return CV_CMPLTU_H;
    INSTR_CV_CMPLTU_SC_H:      return CV_CMPLTU_SC_H;
    INSTR_CV_CMPLTU_SCI_H:     return CV_CMPLTU_SCI_H;
    INSTR_CV_CMPLTU_B:         return CV_CMPLTU_B;
    INSTR_CV_CMPLTU_SC_B:      return CV_CMPLTU_SC_B;
    INSTR_CV_CMPLTU_SCI_B:     return CV_CMPLTU_SCI_B;
    INSTR_CV_CMPLEU_H:         return CV_CMPLEU_H;
    INSTR_CV_CMPLEU_SC_H:      return CV_CMPLEU_SC_H;
    INSTR_CV_CMPLEU_SCI_H:     return CV_CMPLEU_SCI_H;
    INSTR_CV_CMPLEU_B:         return CV_CMPLEU_B;
    INSTR_CV_CMPLEU_SC_B:      return CV_CMPLEU_SC_B;
    INSTR_CV_CMPLEU_SCI_B:     return CV_CMPLEU_SCI_B;
    INSTR_CV_CPLXMUL_R:        return CV_CPLXMUL_R;
    INSTR_CV_CPLXMUL_R_DIV2:   return CV_CPLXMUL_R_DIV2;
    INSTR_CV_CPLXMUL_R_DIV4:   return CV_CPLXMUL_R_DIV4;
    INSTR_CV_CPLXMUL_R_DIV8:   return CV_CPLXMUL_R_DIV8;
    INSTR_CV_CPLXMUL_I:        return CV_CPLXMUL_I;
    INSTR_CV_CPLXMUL_I_DIV2:   return CV_CPLXMUL_I_DIV2;
    INSTR_CV_CPLXMUL_I_DIV4:   return CV_CPLXMUL_I_DIV4;
    INSTR_CV_CPLXMUL_I_DIV8:   return CV_CPLXMUL_I_DIV8;
    INSTR_CV_CPLXCONJ:         return CV_CPLXCONJ;
    INSTR_CV_SUBROTMJ:         return CV_SUBROTMJ;
    INSTR_CV_SUBROTMJ_DIV2:    return CV_SUBROTMJ_DIV2;
    INSTR_CV_SUBROTMJ_DIV4:    return CV_SUBROTMJ_DIV4;
    INSTR_CV_SUBROTMJ_DIV8:    return CV_SUBROTMJ_DIV8;
    INSTR_CV_ADD_DIV2:         return CV_ADD_DIV2;
    INSTR_CV_ADD_DIV4:         return CV_ADD_DIV4;
    INSTR_CV_ADD_DIV8:         return CV_ADD_DIV8;
    INSTR_CV_SUB_DIV2:         return CV_SUB_DIV2;
    INSTR_CV_SUB_DIV4:         return CV_SUB_DIV4;
    INSTR_CV_SUB_DIV8:         return CV_SUB_DIV8;
    default:                   return RV32X_UNKNOWN;
  endcase
endfunction : decode_rv32x

// Ranges of a sign-extended immediate field
`define RV32X_CP_SIMM(NAME, FIELD, WIDTH, ENABLED) \
  NAME: coverpoint FIELD { \
    ignore_bins IGN_OFF = {[0:(1<<WIDTH)-1]} with (!(ENABLED)); \
    bins ZERO     = {0}; \
    bins POSITIVE = {[1:(1<<(WIDTH-1))-2]}; \
    bins MAX_POS  = {(1<<(WIDTH-1))-1}; \
    bins MIN_NEG  = {1<<(WIDTH-1)}; \
    bins NEGATIVE = {[(1<<(WIDTH-1))+1:(1<<WIDTH)-2]}; \
    bins ALL_ONES = {(1<<WIDTH)-1}; \
  }

// Ranges of a zero-extended immediate field
`define RV32X_CP_UIMM(NAME, FIELD, WIDTH, ENABLED) \
  NAME: coverpoint FIELD { \
    ignore_bins IGN_OFF = {[0:(1<<WIDTH)-1]} with (!(ENABLED)); \
    bins ZERO = {0}; \
    bins LOW  = {[1:(1<<(WIDTH-1))-1]}; \
    bins HIGH = {[1<<(WIDTH-1):(1<<WIDTH)-2]}; \
    bins MAX  = {(1<<WIDTH)-1}; \
  }

// Every value of a field, up to MAX
`define RV32X_CP_FIELD(NAME, FIELD, MAX, ENABLED) \
  NAME: coverpoint FIELD { \
    ignore_bins IGN_OFF = {[0:MAX]} with (!(ENABLED)); \
    bins VALUE[] = {[0:MAX]}; \
  }

`define RV32X_CP_SIGN(NAME, FIELD, ENABLED) \
  NAME: coverpoint get_rv32x_sign(instr.FIELD) { \
    ignore_bins IGN_OFF = {[RV32X_ZERO:RV32X_NEGATIVE]} with (!(ENABLED)); \
  }

// Extremes of a scalar source
`define RV32X_CP_SPECIAL(NAME, FIELD, ENABLED) \
  NAME: coverpoint instr.FIELD { \
    ignore_bins IGN_OFF = {32'h0000_0001, 32'h7FFF_FFFF, 32'h8000_0000, 32'hFFFF_FFFF} with (!(ENABLED)); \
    bins ONE      = {32'h0000_0001}; \
    bins MAX_POS  = {32'h7FFF_FFFF}; \
    bins MIN_NEG  = {32'h8000_0000}; \
    bins ALL_ONES = {32'hFFFF_FFFF}; \
  }

// Sign of the result written to rd, unless rd is x0. NONNEG means the result is never negative.
`define RV32X_CP_RD_SIGN(NONNEG) \
  cp_rd_value: coverpoint get_rv32x_sign(instr.rd_value) iff (instr.rd != 0) { \
    ignore_bins NEG_OFF = {RV32X_NEGATIVE} with (NONNEG); \
  }

`define RV32X_CP_LANE(OP, FIELD, LANE, ENABLED) \
  cp_``OP``_lane``LANE: coverpoint get_rv32x_lane_sign(instr.FIELD, lanes, LANE) { \
    ignore_bins IGN_OFF = {[RV32X_ZERO:RV32X_NEGATIVE]} with (!(ENABLED)); \
  }

`define RV32X_CP_LANES(OP, FIELD, NB_LANES) \
  `RV32X_CP_LANE(OP, FIELD, 0, (NB_LANES) > 0) \
  `RV32X_CP_LANE(OP, FIELD, 1, (NB_LANES) > 1) \
  `RV32X_CP_LANE(OP, FIELD, 2, (NB_LANES) > 2) \
  `RV32X_CP_LANE(OP, FIELD, 3, (NB_LANES) > 3)

`define RV32X_CP_ALIGN(ALIGN_HALF, ALIGN_WORD) \
  cp_align_halfword: coverpoint instr.mem_addr[0] { \
    ignore_bins IGN_OFF = {[0:1]} with (!(ALIGN_HALF)); \
    bins ALIGNED   = {0}; \
    bins UNALIGNED = {1}; \
  } \
  cp_align_word: coverpoint instr.mem_addr { \
    ignore_bins IGN_OFF = {[0:3]} with (!(ALIGN_WORD)); \
    bins ALIGNED     = {0}; \
    bins UNALIGNED[] = {[1:3]}; \
  }

// cv.lb, cv.lbu, cv.lh, cv.lhu and cv.lw, post-increment with an immediate or register
// offset and register-register, and cv.elw. With post-increment and rd = rs1 the loaded
// data is written last. rd_nonneg is set for the zero-extending loads.
covergroup cg_rv32x_load(string name, bit has_rs2, bit rd_nonneg, bit align_half, bit align_word)
  with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rd:  coverpoint instr.rd;
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (!has_rs2);
  }

  cp_rd_eq_rs1: coverpoint (instr.rd == instr.rs1) {
    bins EQUAL = {1};
  }

  `RV32X_CP_SIMM(cp_imm, instr.imm12, 12, !has_rs2)
  `RV32X_CP_SIGN(cp_rs2_value, rs2_value, has_rs2)
  `RV32X_CP_SPECIAL(cp_rs2_special, rs2_value, has_rs2)
  `RV32X_CP_RD_SIGN(rd_nonneg)

  `RV32X_CP_ALIGN(align_half, align_word)
endgroup : cg_rv32x_load

// cv.sb, cv.sh and cv.sw, post-increment with an immediate or register offset and
// register-register. The offset register (rs3) is in the rd field.
covergroup cg_rv32x_store(string name, bit has_rs3, bit align_half, bit align_word)
  with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2;
  cp_rs3: coverpoint instr.rs3 {
    ignore_bins IGN_OFF = {[0:31]} with (!has_rs3);
  }

  `RV32X_CP_SIMM(cp_imm, instr.imm12, 12, !has_rs3)
  `RV32X_CP_SIGN(cp_rs2_value, rs2_value, 1)
  `RV32X_CP_SIGN(cp_rs3_value, rs3_value, has_rs3)
  `RV32X_CP_SPECIAL(cp_rs3_special, rs3_value, has_rs3)

  `RV32X_CP_ALIGN(align_half, align_word)
endgroup : cg_rv32x_store

// cv.starti, cv.endi and cv.counti (uimmL), cv.start, cv.end and cv.count (rs1), cv.setupi
// (uimmL, uimmS) and cv.setup (rs1, uimmL). The end offset, uimmS of cv.setupi and uimmL
// of cv.setup (uimml_is_end), is at least 3. With end = PC + 4 * offset and
// start = PC + 4, a body of at least 3 instructions ending on an aligned address has
// end - start >= 8.
covergroup cg_rv32x_hwloop(string name, bit has_rs1, bit has_uimml, bit uimml_is_end, bit has_uimms)
  with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1 {
    ignore_bins IGN_OFF = {[0:31]} with (!has_rs1);
  }

  `RV32X_CP_SIGN(cp_rs1_value, rs1_value, has_rs1)
  `RV32X_CP_SPECIAL(cp_rs1_special, rs1_value, has_rs1)

  cp_uimml: coverpoint instr.imm12 {
    ignore_bins IGN_OFF    = {[0:4095]} with (!has_uimml);
    ignore_bins SHORT_BODY = {[0:2]} with (uimml_is_end);
    bins ZERO = {0};
    bins LOW  = {[1:2047]};
    bins HIGH = {[2048:4094]};
    bins MAX  = {4095};
  }

  cp_uimms: coverpoint instr.is2 {
    ignore_bins IGN_OFF = {[3:31]} with (!has_uimms);
    bins VALUE[] = {[3:31]};
  }
endgroup : cg_rv32x_hwloop

// Scalar ALU, bit manipulation and MAC operations, with one or two register sources, the
// Is2 and Is3 fields, and rd as a third source. is3_max is 3 for cv.bitrev, whose Is3 has
// 2 bits. rd_nonneg is set when the result is never negative (count, compare, zero
// extension, unsigned clip).
covergroup cg_rv32x_alu(string name, bit has_rs2, bit has_is2, bit has_is3, int unsigned is3_max,
                        bit has_rs3, bit rd_nonneg)
  with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (!has_rs2);
  }
  cp_rd:  coverpoint instr.rd;

  cp_rd_eq_rs1: coverpoint (instr.rd == instr.rs1) {
    bins EQUAL = {1};
  }
  cp_rd_eq_rs2: coverpoint (instr.rd == instr.rs2) {
    ignore_bins IGN_OFF = {[0:1]} with (!has_rs2);
    bins EQUAL = {1};
  }

  `RV32X_CP_FIELD(cp_is2, instr.is2, 31, has_is2)
  `RV32X_CP_FIELD(cp_is3, instr.is3, is3_max, has_is3)

  `RV32X_CP_SIGN(cp_rs1_value, rs1_value, 1)
  `RV32X_CP_SPECIAL(cp_rs1_special, rs1_value, 1)
  `RV32X_CP_SIGN(cp_rs2_value, rs2_value, has_rs2)
  `RV32X_CP_SPECIAL(cp_rs2_special, rs2_value, has_rs2)
  `RV32X_CP_SIGN(cp_rs3_value, rs3_value, has_rs3)

  `RV32X_CP_RD_SIGN(rd_nonneg)
endgroup : cg_rv32x_alu

// cv.beqimm, cv.bneimm
covergroup cg_rv32x_branch(string name) with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;

  `RV32X_CP_FIELD(cp_imm5, instr.is2, 31, 1)

  `RV32X_CP_SIGN(cp_rs1_value, rs1_value, 1)
  `RV32X_CP_SPECIAL(cp_rs1_special, rs1_value, 1)

  cp_taken: coverpoint instr.taken {
    bins NOT_TAKEN = {0};
    bins TAKEN     = {1};
  }
endgroup : cg_rv32x_branch

// Lane of the SIMD destination. A compare sets each lane to all zeros or all ones.
`define RV32X_CP_RD_LANE(LANE) \
  cp_rd_lane``LANE: coverpoint get_rv32x_lane_sign(instr.rd_value, lanes, LANE) iff (instr.rd != 0) { \
    ignore_bins IGN_OFF = {[RV32X_ZERO:RV32X_NEGATIVE]} with ((rd_kind == RV32X_RD_SCALAR) || (LANE >= lanes)); \
    ignore_bins POS_OFF = {RV32X_POSITIVE} with (rd_kind == RV32X_RD_MASK); \
  }

// SIMD operations. lanes is 2 for .h and 4 for .b. rs2_lanes is 0 for .sci, which has one
// register source, 1 for .sc and lanes for a vector. imm6_max is the largest value of an
// RV32X_IMM6_FIELD Imm6.
covergroup cg_rv32x_simd(string name, int unsigned lanes, int unsigned rs2_lanes,
                         rv32x_imm6_kind_t imm6_kind, int unsigned imm6_max,
                         rv32x_rd_kind_t rd_kind, rv32x_rs3_kind_t rs3_kind)
  with function sample(rv32x_instr_t instr);
  option.per_instance = 1;
  option.name = name;

  cp_rs1: coverpoint instr.rs1;
  cp_rs2: coverpoint instr.rs2 {
    ignore_bins IGN_OFF = {[0:31]} with (rs2_lanes == 0);
  }
  cp_rd:  coverpoint instr.rd;

  cp_rd_eq_rs1: coverpoint (instr.rd == instr.rs1) {
    bins EQUAL = {1};
  }
  cp_rd_eq_rs2: coverpoint (instr.rd == instr.rs2) {
    ignore_bins IGN_OFF = {[0:1]} with (rs2_lanes == 0);
    bins EQUAL = {1};
  }

  `RV32X_CP_SIMM(cp_imm6, instr.imm6, 6, imm6_kind == RV32X_IMM6_SIGNED)
  `RV32X_CP_UIMM(cp_uimm6, instr.imm6, 6, imm6_kind == RV32X_IMM6_UNSIGNED)
  `RV32X_CP_FIELD(cp_imm6_value, instr.imm6, imm6_max, imm6_kind == RV32X_IMM6_FIELD)

  `RV32X_CP_LANES(rs1, rs1_value, lanes)
  `RV32X_CP_LANES(rs2, rs2_value, rs2_lanes)
  `RV32X_CP_LANES(rs3, rs3_value, (rs3_kind == RV32X_RS3_LANES) ? lanes : 0)
  `RV32X_CP_SIGN(cp_rs3_value, rs3_value, rs3_kind == RV32X_RS3_SCALAR)

  `RV32X_CP_RD_LANE(0)
  `RV32X_CP_RD_LANE(1)
  `RV32X_CP_RD_LANE(2)
  `RV32X_CP_RD_LANE(3)

  cp_rd_value: coverpoint get_rv32x_sign(instr.rd_value) iff (instr.rd != 0) {
    ignore_bins IGN_OFF = {[RV32X_ZERO:RV32X_NEGATIVE]} with (rd_kind != RV32X_RD_SCALAR);
  }
endgroup : cg_rv32x_simd


class uvme_rv32x_isa_covg extends uvm_component;

  uvme_cv32e40p_cfg_c cfg;

  uvm_analysis_imp#(uvma_isacov_mon_trn_c, uvme_rv32x_isa_covg) mon_trn_export;

  cg_rv32x_load   load_cg[rv32x_instr_name_t];
  cg_rv32x_store  store_cg[rv32x_instr_name_t];
  cg_rv32x_hwloop hwloop_cg[rv32x_instr_name_t];
  cg_rv32x_alu    alu_cg[rv32x_instr_name_t];
  cg_rv32x_branch branch_cg[rv32x_instr_name_t];
  cg_rv32x_simd   simd_cg[rv32x_instr_name_t];

  `uvm_component_utils_begin(uvme_rv32x_isa_covg)
    `uvm_field_object(cfg, UVM_DEFAULT)
  `uvm_component_utils_end

  extern function new(string name = "rv32x_isa_covg", uvm_component parent = null);
  extern function void build_phase(uvm_phase phase);
  extern function void build_scalar_cg();
  extern function void build_simd_cg();
  extern function string cg_name(rv32x_instr_name_t name);
  extern function void write(uvma_isacov_mon_trn_c trn);
  extern function rv32x_instr_t get_instr(uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi, rv32x_instr_name_t name);

endclass : uvme_rv32x_isa_covg


function uvme_rv32x_isa_covg::new(string name = "rv32x_isa_covg", uvm_component parent = null);

  super.new(name, parent);

  mon_trn_export = new("mon_trn_export", this);

endfunction : new


function void uvme_rv32x_isa_covg::build_phase(uvm_phase phase);

  super.build_phase(phase);

  void'(uvm_config_db#(uvme_cv32e40p_cfg_c)::get(this, "", "cfg", cfg));
  if (cfg == null) begin
    `uvm_fatal("RV32XISACOVG", "Configuration handle is null")
  end

  build_scalar_cg();
  build_simd_cg();

endfunction : build_phase


function void uvme_rv32x_isa_covg::build_scalar_cg();

  rv32x_instr_name_t alu_r1[]      = '{CV_ABS, CV_EXTHS, CV_EXTBS};
  rv32x_instr_name_t alu_r1_nn[]   = '{CV_FF1, CV_FL1, CV_CLB, CV_CNT, CV_EXTHZ, CV_EXTBZ};
  rv32x_instr_name_t alu_r2[]      = '{CV_MIN, CV_MINU, CV_MAX, CV_MAXU, CV_ROR,
                                       CV_EXTRACTR, CV_EXTRACTUR, CV_BCLRR, CV_BSETR, CV_CLIPR};
  rv32x_instr_name_t alu_r2_nn[]   = '{CV_SLE, CV_SLEU, CV_CLIPUR};
  rv32x_instr_name_t alu_r3[]      = '{CV_INSERTR, CV_MAC, CV_MSU,
                                       CV_ADDNR, CV_ADDUNR, CV_ADDRNR, CV_ADDURNR,
                                       CV_SUBNR, CV_SUBUNR, CV_SUBRNR, CV_SUBURNR};
  rv32x_instr_name_t alu_is2_is3[] = '{CV_EXTRACT, CV_EXTRACTU, CV_BCLR, CV_BSET};
  rv32x_instr_name_t alu_r2_is3[]  = '{CV_ADDN, CV_ADDUN, CV_ADDRN, CV_ADDURN,
                                       CV_SUBN, CV_SUBUN, CV_SUBRN, CV_SUBURN,
                                       CV_MULSN, CV_MULHHSN, CV_MULSRN, CV_MULHHSRN,
                                       CV_MULUN, CV_MULHHUN, CV_MULURN, CV_MULHHURN};
  rv32x_instr_name_t alu_r3_is3[]  = '{CV_MACSN, CV_MACHHSN, CV_MACSRN, CV_MACHHSRN,
                                       CV_MACUN, CV_MACHHUN, CV_MACURN, CV_MACHHURN};

  //                                                   has_rs2       rd_nonneg       align_half       align_word
  load_cg[CV_LB_PI_RI]  = new(cg_name(CV_LB_PI_RI),  .has_rs2(0), .rd_nonneg(0), .align_half(0), .align_word(0));
  load_cg[CV_LBU_PI_RI] = new(cg_name(CV_LBU_PI_RI), .has_rs2(0), .rd_nonneg(1), .align_half(0), .align_word(0));
  load_cg[CV_LH_PI_RI]  = new(cg_name(CV_LH_PI_RI),  .has_rs2(0), .rd_nonneg(0), .align_half(1), .align_word(0));
  load_cg[CV_LHU_PI_RI] = new(cg_name(CV_LHU_PI_RI), .has_rs2(0), .rd_nonneg(1), .align_half(1), .align_word(0));
  load_cg[CV_LW_PI_RI]  = new(cg_name(CV_LW_PI_RI),  .has_rs2(0), .rd_nonneg(0), .align_half(0), .align_word(1));
  load_cg[CV_LB_PI_RR]  = new(cg_name(CV_LB_PI_RR),  .has_rs2(1), .rd_nonneg(0), .align_half(0), .align_word(0));
  load_cg[CV_LBU_PI_RR] = new(cg_name(CV_LBU_PI_RR), .has_rs2(1), .rd_nonneg(1), .align_half(0), .align_word(0));
  load_cg[CV_LH_PI_RR]  = new(cg_name(CV_LH_PI_RR),  .has_rs2(1), .rd_nonneg(0), .align_half(1), .align_word(0));
  load_cg[CV_LHU_PI_RR] = new(cg_name(CV_LHU_PI_RR), .has_rs2(1), .rd_nonneg(1), .align_half(1), .align_word(0));
  load_cg[CV_LW_PI_RR]  = new(cg_name(CV_LW_PI_RR),  .has_rs2(1), .rd_nonneg(0), .align_half(0), .align_word(1));
  load_cg[CV_LB_RR]     = new(cg_name(CV_LB_RR),     .has_rs2(1), .rd_nonneg(0), .align_half(0), .align_word(0));
  load_cg[CV_LBU_RR]    = new(cg_name(CV_LBU_RR),    .has_rs2(1), .rd_nonneg(1), .align_half(0), .align_word(0));
  load_cg[CV_LH_RR]     = new(cg_name(CV_LH_RR),     .has_rs2(1), .rd_nonneg(0), .align_half(1), .align_word(0));
  load_cg[CV_LHU_RR]    = new(cg_name(CV_LHU_RR),    .has_rs2(1), .rd_nonneg(1), .align_half(1), .align_word(0));
  load_cg[CV_LW_RR]     = new(cg_name(CV_LW_RR),     .has_rs2(1), .rd_nonneg(0), .align_half(0), .align_word(1));
`ifdef CLUSTER
  load_cg[CV_ELW]       = new(cg_name(CV_ELW),       .has_rs2(0), .rd_nonneg(0), .align_half(0), .align_word(1));
`endif

  //                                                 has_rs3       align_half       align_word
  store_cg[CV_SB_PI_RI] = new(cg_name(CV_SB_PI_RI), .has_rs3(0), .align_half(0), .align_word(0));
  store_cg[CV_SH_PI_RI] = new(cg_name(CV_SH_PI_RI), .has_rs3(0), .align_half(1), .align_word(0));
  store_cg[CV_SW_PI_RI] = new(cg_name(CV_SW_PI_RI), .has_rs3(0), .align_half(0), .align_word(1));
  store_cg[CV_SB_PI_RR] = new(cg_name(CV_SB_PI_RR), .has_rs3(1), .align_half(0), .align_word(0));
  store_cg[CV_SH_PI_RR] = new(cg_name(CV_SH_PI_RR), .has_rs3(1), .align_half(1), .align_word(0));
  store_cg[CV_SW_PI_RR] = new(cg_name(CV_SW_PI_RR), .has_rs3(1), .align_half(0), .align_word(1));
  store_cg[CV_SB_RR]    = new(cg_name(CV_SB_RR),    .has_rs3(1), .align_half(0), .align_word(0));
  store_cg[CV_SH_RR]    = new(cg_name(CV_SH_RR),    .has_rs3(1), .align_half(1), .align_word(0));
  store_cg[CV_SW_RR]    = new(cg_name(CV_SW_RR),    .has_rs3(1), .align_half(0), .align_word(1));

  //                                                   has_rs1       has_uimml       uimml_is_end       has_uimms
  hwloop_cg[CV_STARTI_0] = new(cg_name(CV_STARTI_0), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_STARTI_1] = new(cg_name(CV_STARTI_1), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_START_0]  = new(cg_name(CV_START_0),  .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_START_1]  = new(cg_name(CV_START_1),  .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_ENDI_0]   = new(cg_name(CV_ENDI_0),   .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_ENDI_1]   = new(cg_name(CV_ENDI_1),   .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_END_0]    = new(cg_name(CV_END_0),    .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_END_1]    = new(cg_name(CV_END_1),    .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_COUNTI_0] = new(cg_name(CV_COUNTI_0), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_COUNTI_1] = new(cg_name(CV_COUNTI_1), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_COUNT_0]  = new(cg_name(CV_COUNT_0),  .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_COUNT_1]  = new(cg_name(CV_COUNT_1),  .has_rs1(1), .has_uimml(0), .uimml_is_end(0), .has_uimms(0));
  hwloop_cg[CV_SETUPI_0] = new(cg_name(CV_SETUPI_0), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(1));
  hwloop_cg[CV_SETUPI_1] = new(cg_name(CV_SETUPI_1), .has_rs1(0), .has_uimml(1), .uimml_is_end(0), .has_uimms(1));
  hwloop_cg[CV_SETUP_0]  = new(cg_name(CV_SETUP_0),  .has_rs1(1), .has_uimml(1), .uimml_is_end(1), .has_uimms(0));
  hwloop_cg[CV_SETUP_1]  = new(cg_name(CV_SETUP_1),  .has_rs1(1), .has_uimml(1), .uimml_is_end(1), .has_uimms(0));

  foreach (alu_r1[i])
    alu_cg[alu_r1[i]] = new(cg_name(alu_r1[i]), .has_rs2(0), .has_is2(0), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(0));
  foreach (alu_r1_nn[i])
    alu_cg[alu_r1_nn[i]] = new(cg_name(alu_r1_nn[i]), .has_rs2(0), .has_is2(0), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(1));
  foreach (alu_r2[i])
    alu_cg[alu_r2[i]] = new(cg_name(alu_r2[i]), .has_rs2(1), .has_is2(0), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(0));
  foreach (alu_r2_nn[i])
    alu_cg[alu_r2_nn[i]] = new(cg_name(alu_r2_nn[i]), .has_rs2(1), .has_is2(0), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(1));
  foreach (alu_r3[i])
    alu_cg[alu_r3[i]] = new(cg_name(alu_r3[i]), .has_rs2(1), .has_is2(0), .has_is3(0), .is3_max(31), .has_rs3(1), .rd_nonneg(0));
  foreach (alu_is2_is3[i])
    alu_cg[alu_is2_is3[i]] = new(cg_name(alu_is2_is3[i]), .has_rs2(0), .has_is2(1), .has_is3(1), .is3_max(31), .has_rs3(0), .rd_nonneg(0));
  foreach (alu_r2_is3[i])
    alu_cg[alu_r2_is3[i]] = new(cg_name(alu_r2_is3[i]), .has_rs2(1), .has_is2(0), .has_is3(1), .is3_max(31), .has_rs3(0), .rd_nonneg(0));
  foreach (alu_r3_is3[i])
    alu_cg[alu_r3_is3[i]] = new(cg_name(alu_r3_is3[i]), .has_rs2(1), .has_is2(0), .has_is3(1), .is3_max(31), .has_rs3(1), .rd_nonneg(0));
  alu_cg[CV_INSERT] = new(cg_name(CV_INSERT), .has_rs2(0), .has_is2(1), .has_is3(1), .is3_max(31), .has_rs3(1), .rd_nonneg(0));
  alu_cg[CV_BITREV] = new(cg_name(CV_BITREV), .has_rs2(0), .has_is2(1), .has_is3(1), .is3_max(3),  .has_rs3(0), .rd_nonneg(0));
  alu_cg[CV_CLIP]   = new(cg_name(CV_CLIP),   .has_rs2(0), .has_is2(1), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(0));
  alu_cg[CV_CLIPU]  = new(cg_name(CV_CLIPU),  .has_rs2(0), .has_is2(1), .has_is3(0), .is3_max(31), .has_rs3(0), .rd_nonneg(1));

  branch_cg[CV_BEQIMM] = new(cg_name(CV_BEQIMM));
  branch_cg[CV_BNEIMM] = new(cg_name(CV_BNEIMM));

endfunction : build_scalar_cg


function void uvme_rv32x_isa_covg::build_simd_cg();

  // The six variants of these operations (.h, .sc.h, .sci.h, .b, .sc.b, .sci.b) follow the
  // .h one in rv32x_instr_name_t, as in uvme_cv32e40p_constants.sv
  rv32x_instr_name_t lanes_op[]    = '{CV_ADD_H, CV_SUB_H, CV_AVG_H, CV_AVGU_H, CV_MIN_H, CV_MINU_H,
                                       CV_MAX_H, CV_MAXU_H, CV_SRL_H, CV_SRA_H, CV_SLL_H,
                                       CV_OR_H, CV_XOR_H, CV_AND_H};
  rv32x_instr_name_t mask_op[]     = '{CV_CMPEQ_H, CV_CMPNE_H, CV_CMPGT_H, CV_CMPGE_H, CV_CMPLT_H, CV_CMPLE_H,
                                       CV_CMPGTU_H, CV_CMPGEU_H, CV_CMPLTU_H, CV_CMPLEU_H};
  rv32x_instr_name_t dot_op[]      = '{CV_DOTUP_H, CV_DOTUSP_H, CV_DOTSP_H};
  rv32x_instr_name_t sdot_op[]     = '{CV_SDOTUP_H, CV_SDOTUSP_H, CV_SDOTSP_H};
  // The cv32e40p decoder zero-extends Imm6 of the .sci variant for these operations, takes
  // a shift amount of 4 (.h) or 3 (.b) bits for the shifts, and sign-extends it for the
  // others. The specification zero-extends the shift amount and the decoder sign-extends
  // it, which gives the same value because the upper Imm6 bits must be 0.
  rv32x_instr_name_t unsigned_op[] = '{CV_AVGU_H, CV_MINU_H, CV_MAXU_H, CV_CMPGTU_H, CV_CMPGEU_H,
                                       CV_CMPLTU_H, CV_CMPLEU_H, CV_DOTUP_H, CV_SDOTUP_H};
  rv32x_instr_name_t shift_op[]    = '{CV_SRL_H, CV_SRA_H, CV_SLL_H};
  string             variant[6]    = '{"_H", "_SC_H", "_SCI_H", "_B", "_SC_B", "_SCI_B"};
  rv32x_instr_name_t cplx_rd[]     = '{CV_CPLXMUL_R, CV_CPLXMUL_R_DIV2, CV_CPLXMUL_R_DIV4, CV_CPLXMUL_R_DIV8,
                                       CV_CPLXMUL_I, CV_CPLXMUL_I_DIV2, CV_CPLXMUL_I_DIV4, CV_CPLXMUL_I_DIV8};
  rv32x_instr_name_t cplx[]        = '{CV_SUBROTMJ, CV_SUBROTMJ_DIV2, CV_SUBROTMJ_DIV4, CV_SUBROTMJ_DIV8,
                                       CV_ADD_DIV2, CV_ADD_DIV4, CV_ADD_DIV8,
                                       CV_SUB_DIV2, CV_SUB_DIV4, CV_SUB_DIV8, CV_PACK, CV_PACK_H};
  rv32x_instr_name_t ops[$];
  rv32x_rd_kind_t    rd_kind[$];
  rv32x_rs3_kind_t   rs3_kind[$];

  foreach (lanes_op[i]) begin ops.push_back(lanes_op[i]); rd_kind.push_back(RV32X_RD_LANES);  rs3_kind.push_back(RV32X_RS3_NONE);   end
  foreach (mask_op[i])  begin ops.push_back(mask_op[i]);  rd_kind.push_back(RV32X_RD_MASK);   rs3_kind.push_back(RV32X_RS3_NONE);   end
  foreach (dot_op[i])   begin ops.push_back(dot_op[i]);   rd_kind.push_back(RV32X_RD_SCALAR); rs3_kind.push_back(RV32X_RS3_NONE);   end
  foreach (sdot_op[i])  begin ops.push_back(sdot_op[i]);  rd_kind.push_back(RV32X_RD_SCALAR); rs3_kind.push_back(RV32X_RS3_SCALAR); end

  foreach (ops[i]) begin
    string            op        = ops[i].name();
    rv32x_imm6_kind_t imm6_kind = RV32X_IMM6_SIGNED;

    if (ops[i] inside {unsigned_op})
      imm6_kind = RV32X_IMM6_UNSIGNED;
    if (ops[i] inside {shift_op})
      imm6_kind = RV32X_IMM6_FIELD;
    op = op.substr(0, op.len() - 3);  // without "_H"
    for (int v = 0; v < 6; v++) begin
      rv32x_instr_name_t n     = rv32x_instr_name_t'(ops[i] + v);
      int unsigned       lanes = (v < 3) ? 2 : 4;

      if (n.name() != {op, variant[v]}) begin
        `uvm_fatal("RV32XISACOVG", $sformatf("%s is not the %s variant of %s", n.name(), variant[v], ops[i].name()))
      end
      //                                       rs2_lanes  imm6_kind        imm6_max          rd_kind     rs3_kind
      case (v % 3)
        0: simd_cg[n] = new(cg_name(n), lanes, lanes,     RV32X_IMM6_NONE, 0,                rd_kind[i], rs3_kind[i]);
        1: simd_cg[n] = new(cg_name(n), lanes, 1,         RV32X_IMM6_NONE, 0,                rd_kind[i], rs3_kind[i]);
        2: simd_cg[n] = new(cg_name(n), lanes, 0,         imm6_kind,       (32 / lanes) - 1, rd_kind[i], rs3_kind[i]);
      endcase
    end
  end

  //                                                 lanes rs2_lanes imm6_kind      imm6_max rd_kind        rs3_kind
  foreach (cplx_rd[i])
    simd_cg[cplx_rd[i]] = new(cg_name(cplx_rd[i]), 2, 2, RV32X_IMM6_NONE, 0, RV32X_RD_LANES, RV32X_RS3_LANES);
  foreach (cplx[i])
    simd_cg[cplx[i]]    = new(cg_name(cplx[i]),    2, 2, RV32X_IMM6_NONE, 0, RV32X_RD_LANES, RV32X_RS3_NONE);

  // Imm6 is a lane index for extract and insert, and the lane selectors for the .sci shuffles
  //                                                             lanes rs2_lanes imm6_kind   imm6_max rd_kind      rs3_kind
  simd_cg[CV_ABS_H]           = new(cg_name(CV_ABS_H),           2, 0, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_ABS_B]           = new(cg_name(CV_ABS_B),           4, 0, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_CPLXCONJ]        = new(cg_name(CV_CPLXCONJ),        2, 0, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_EXTRACT_H]       = new(cg_name(CV_EXTRACT_H),       2, 0, RV32X_IMM6_FIELD, 1,  RV32X_RD_SCALAR, RV32X_RS3_NONE);
  simd_cg[CV_EXTRACT_B]       = new(cg_name(CV_EXTRACT_B),       4, 0, RV32X_IMM6_FIELD, 3,  RV32X_RD_SCALAR, RV32X_RS3_NONE);
  simd_cg[CV_EXTRACTU_H]      = new(cg_name(CV_EXTRACTU_H),      2, 0, RV32X_IMM6_FIELD, 1,  RV32X_RD_SCALAR, RV32X_RS3_NONE);
  simd_cg[CV_EXTRACTU_B]      = new(cg_name(CV_EXTRACTU_B),      4, 0, RV32X_IMM6_FIELD, 3,  RV32X_RD_SCALAR, RV32X_RS3_NONE);
  simd_cg[CV_INSERT_H]        = new(cg_name(CV_INSERT_H),        2, 0, RV32X_IMM6_FIELD, 1,  RV32X_RD_LANES,  RV32X_RS3_LANES);
  simd_cg[CV_INSERT_B]        = new(cg_name(CV_INSERT_B),        4, 0, RV32X_IMM6_FIELD, 3,  RV32X_RD_LANES,  RV32X_RS3_LANES);
  simd_cg[CV_SHUFFLE_H]       = new(cg_name(CV_SHUFFLE_H),       2, 2, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLE_B]       = new(cg_name(CV_SHUFFLE_B),       4, 4, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLE_SCI_H]   = new(cg_name(CV_SHUFFLE_SCI_H),   2, 0, RV32X_IMM6_FIELD, 3,  RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLEI0_SCI_B] = new(cg_name(CV_SHUFFLEI0_SCI_B), 4, 0, RV32X_IMM6_FIELD, 63, RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLEI1_SCI_B] = new(cg_name(CV_SHUFFLEI1_SCI_B), 4, 0, RV32X_IMM6_FIELD, 63, RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLEI2_SCI_B] = new(cg_name(CV_SHUFFLEI2_SCI_B), 4, 0, RV32X_IMM6_FIELD, 63, RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLEI3_SCI_B] = new(cg_name(CV_SHUFFLEI3_SCI_B), 4, 0, RV32X_IMM6_FIELD, 63, RV32X_RD_LANES,  RV32X_RS3_NONE);
  simd_cg[CV_SHUFFLE2_H]      = new(cg_name(CV_SHUFFLE2_H),      2, 2, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_LANES);
  simd_cg[CV_SHUFFLE2_B]      = new(cg_name(CV_SHUFFLE2_B),      4, 4, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_LANES);
  simd_cg[CV_PACKHI_B]        = new(cg_name(CV_PACKHI_B),        4, 4, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_LANES);
  simd_cg[CV_PACKLO_B]        = new(cg_name(CV_PACKLO_B),        4, 4, RV32X_IMM6_NONE,  0,  RV32X_RD_LANES,  RV32X_RS3_LANES);

endfunction : build_simd_cg


function string uvme_rv32x_isa_covg::cg_name(rv32x_instr_name_t name);

  return $sformatf("rv32x_%s_cg", name.name().tolower());

endfunction : cg_name


function void uvme_rv32x_isa_covg::write(uvma_isacov_mon_trn_c trn);

  uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi = trn.instr.rvfi;
  rv32x_instr_name_t                     name;
  rv32x_instr_t                          instr;

  // Same filter as uvma_isacov, which keeps the instructions retired without a trap and
  // the single-stepped ones without an exception.
  if (!((rvfi.trap[0] == 0) || ((rvfi.trap[11:9] == 4) && (rvfi.trap[1] == 0))))
    return;

  name = decode_rv32x(rvfi.insn);
  if (name == RV32X_UNKNOWN)
    return;

  instr = get_instr(rvfi, name);

  if (load_cg.exists(name))   load_cg[name].sample(instr);
  if (store_cg.exists(name))  store_cg[name].sample(instr);
  if (hwloop_cg.exists(name)) hwloop_cg[name].sample(instr);
  if (alu_cg.exists(name))    alu_cg[name].sample(instr);
  if (branch_cg.exists(name)) branch_cg[name].sample(instr);
  if (simd_cg.exists(name))   simd_cg[name].sample(instr);

endfunction : write


function rv32x_instr_t uvme_rv32x_isa_covg::get_instr(uvma_rvfi_instr_seq_item_c#(ILEN,XLEN) rvfi, rv32x_instr_name_t name);

  bit [31:0]    insn = rvfi.insn;
  rv32x_instr_t instr;

  instr.name      = name;
  instr.rs1       = insn[19:15];
  instr.rs2       = insn[24:20];
  instr.rd        = insn[11:7];
  instr.rs3       = insn[11:7];
  instr.imm12     = insn[31:20];
  instr.imm6      = {insn[24:20], insn[25]};  // Imm6[0|5:1] in insn[25:20]
  instr.is2       = insn[24:20];
  instr.is3       = insn[29:25];
  instr.mem_addr  = rvfi.mem_addr[1:0];
  instr.rs1_value = rvfi.rs1_rdata;
  instr.rs2_value = rvfi.rs2_rdata;
  instr.rs3_value = rvfi.rs3_rdata;

  // The store offset is split around the rd field, and uimmS of cv.setupi is in the rs1 field
  if (name inside {CV_SB_PI_RI, CV_SH_PI_RI, CV_SW_PI_RI})
    instr.imm12 = {insn[31:25], insn[11:7]};
  if (name inside {CV_SETUPI_0, CV_SETUPI_1})
    instr.is2 = insn[19:15];

  // As uvma_isacov does for the base branches, take the branch outcome from the operands
  if (name == CV_BEQIMM)
    instr.taken = (rvfi.rs1_rdata == {{27{insn[24]}}, insn[24:20]});
  if (name == CV_BNEIMM)
    instr.taken = (rvfi.rs1_rdata != {{27{insn[24]}}, insn[24:20]});

  // A post-increment load writes rs1 in EX (rd1) and the data in WB (rd2). When rd = rs1
  // the WB write comes last and wins.
  if (rvfi.rd2_addr == instr.rd)
    instr.rd_value = rvfi.rd2_wdata;
  else
    instr.rd_value = rvfi.rd1_wdata;

  return instr;

endfunction : get_instr

`undef RV32X_CP_SIMM
`undef RV32X_CP_UIMM
`undef RV32X_CP_FIELD
`undef RV32X_CP_SIGN
`undef RV32X_CP_SPECIAL
`undef RV32X_CP_LANE
`undef RV32X_CP_LANES
`undef RV32X_CP_RD_LANE
`undef RV32X_CP_RD_SIGN
`undef RV32X_CP_ALIGN

`endif // __UVME_RV32X_ISA_COVG_SV__
