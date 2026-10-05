/*
**
** Copyright 2026 Fondazione Chips-IT
**
** Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     https://solderpad.org/licenses/
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**
*******************************************************************************
**
** Runs fcvt.w.s, fcvt.wu.s, fadd.s and fmul.s with rm = dyn under every valid
** frm, and checks that the result and the flags are those of the same
** instruction with the static rounding mode equal to frm. The operands include
** exact ties, where the rounding modes differ. Without F or Zfinx there is
** nothing to check and the test passes.
**
*******************************************************************************
*/

#include <stdint.h>
#include <stdio.h>

#if defined(__riscv_f) || defined(__riscv_zfinx)

// With Zfinx the operands are in the x registers.
#ifdef __riscv_zfinx
#define FREG "r"
#define BIN(op, rm)                                                                                \
  __asm__ volatile("csrw fflags, x0\n\t" op " %0, %2, %3, " rm "\n\tcsrr %1, fflags"              \
                   : "=&r"(*r), "=r"(*f) : "r"(a), "r"(b))
#else
#define FREG "f"
#define BIN(op, rm)                                                                                \
  __asm__ volatile("csrw fflags, x0\n\t" op " ft0, %2, %3, " rm "\n\tfmv.x.w %0, ft0\n\t"          \
                   "csrr %1, fflags" : "=r"(*r), "=r"(*f) : "f"(a), "f"(b) : "ft0")
#endif
#define CVT(op, rm)                                                                                \
  __asm__ volatile("csrw fflags, x0\n\t" op " %0, %2, " rm "\n\tcsrr %1, fflags"                  \
                   : "=r"(*r), "=r"(*f) : FREG(x))
#define STATIC(m, MACRO, op)                                                                       \
  switch (m) {                                                                                     \
  case 0: MACRO(op, "rne"); break;                                                                 \
  case 1: MACRO(op, "rtz"); break;                                                                 \
  case 2: MACRO(op, "rdn"); break;                                                                 \
  case 3: MACRO(op, "rup"); break;                                                                 \
  default: MACRO(op, "rmm"); break;                                                                \
  }

static void cvt_w(int m, float x, uint32_t *r, uint32_t *f) { STATIC(m, CVT, "fcvt.w.s") }
static void cvt_w_dyn(float x, uint32_t *r, uint32_t *f) { CVT("fcvt.w.s", "dyn"); }
static void cvt_wu(int m, float x, uint32_t *r, uint32_t *f) { STATIC(m, CVT, "fcvt.wu.s") }
static void cvt_wu_dyn(float x, uint32_t *r, uint32_t *f) { CVT("fcvt.wu.s", "dyn"); }
static void add(int m, float a, float b, uint32_t *r, uint32_t *f) { STATIC(m, BIN, "fadd.s") }
static void add_dyn(float a, float b, uint32_t *r, uint32_t *f) { BIN("fadd.s", "dyn"); }
static void mul(int m, float a, float b, uint32_t *r, uint32_t *f) { STATIC(m, BIN, "fmul.s") }
static void mul_dyn(float a, float b, uint32_t *r, uint32_t *f) { BIN("fmul.s", "dyn"); }

static const float cvt_in[] = {
  0.5f, 1.5f, 2.5f, -0.5f, -1.5f, -2.5f, 3.5f, 8388607.5f, -8388607.5f,
  0.25f, 0.75f, -0.75f, 2.0f, -3.0f, 1e10f, -1e10f, 2147483520.0f,
};
// Each a op b is either an exact tie between two floats or inexact without a tie.
static const float bin_a[] = { 1.0f, -1.0f, 0x1.000002p0f, 0x1.001p0f, -0x1.001p0f, 3.0f };
static const float bin_b[] = { 0x1p-24f, -0x1p-24f, 0x1p-24f, 0x1.001p0f, 0x1.001p0f, 0x1p-23f };

static int errors;

static void check(const char *op, int m, unsigned i, uint32_t r, uint32_t f, uint32_t er, uint32_t ef)
{
  if (r != er || f != ef) {
    printf("FDYN MISMATCH %s frm=%d vector=%u dyn=%08lx/%02lx static=%08lx/%02lx\n", op, m, i,
           (unsigned long)r, (unsigned long)f, (unsigned long)er, (unsigned long)ef);
    errors++;
  }
}

int main(void)
{
  __asm__ volatile("csrs mstatus, %0" : : "r"(1u << 13));  // FS = Initial
  unsigned n = 0;
  for (int m = 0; m <= 4; m++) {
    __asm__ volatile("csrw frm, %0" : : "r"(m));
    for (unsigned i = 0; i < sizeof(cvt_in) / sizeof(cvt_in[0]); i++) {
      uint32_t r, f, er, ef;
      cvt_w_dyn(cvt_in[i], &r, &f);
      cvt_w(m, cvt_in[i], &er, &ef);
      check("fcvt.w.s", m, i, r, f, er, ef);
      cvt_wu_dyn(cvt_in[i], &r, &f);
      cvt_wu(m, cvt_in[i], &er, &ef);
      check("fcvt.wu.s", m, i, r, f, er, ef);
      n += 2;
    }
    for (unsigned i = 0; i < sizeof(bin_a) / sizeof(bin_a[0]); i++) {
      uint32_t r, f, er, ef;
      add_dyn(bin_a[i], bin_b[i], &r, &f);
      add(m, bin_a[i], bin_b[i], &er, &ef);
      check("fadd.s", m, i, r, f, er, ef);
      mul_dyn(bin_a[i], bin_b[i], &r, &f);
      mul(m, bin_a[i], bin_b[i], &er, &ef);
      check("fmul.s", m, i, r, f, er, ef);
      n += 2;
    }
  }
  printf("FDYN checks=%u mismatches=%d\n", n, errors);
  return errors;
}

#else

int main(void)
{
  printf("FDYN no FPU, nothing to check\n");
  return 0;
}

#endif
