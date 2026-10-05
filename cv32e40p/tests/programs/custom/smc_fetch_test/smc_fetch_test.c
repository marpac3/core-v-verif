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
** Self-modifying code without fence.i. CV32E40P has no instruction cache, so
** a store to an instruction that already ran changes what the next execution
** runs. The routine at label 1 runs once with a nop as its first instruction.
** A store then replaces the nop with addi t1, t1, 1 and the routine runs
** again, so t1 goes from 5 to 6. The two nops after the store keep the jal out
** of ID until the store is granted, because a jump in ID fetches its target
** even while EX waits for the data grant. A store to an instruction that is
** already in the prefetch buffer is not checked, because the RTL then runs the
** old or the new instruction depending on the fetch timing.
**
*******************************************************************************
*/
#include <stdio.h>
#include <stdint.h>

int main(void)
{
  uint32_t r;
  __asm__ volatile(".option push\n\t.option norvc\n\t"
                   "li t1, 5\n\t"
                   "jal ra, 1f\n\t"
                   "li t3, 0x00130313\n\t"
                   "la t0, 1f\n\t"
                   "sw t3, 0(t0)\n\t"
                   "nop\n\t"
                   "nop\n\t"
                   "jal ra, 1f\n\t"
                   "mv %[r], t1\n\t"
                   "j 2f\n"
                   "1:\n\t"
                   "nop\n\t"
                   "ret\n"
                   "2:\n\t"
                   ".option pop"
                   : [r] "=r"(r)
                   :
                   : "memory", "ra", "t0", "t1", "t3");
  printf("SMC r=%u\n", (unsigned)r);
  return r == 6 ? 0 : 1;
}
