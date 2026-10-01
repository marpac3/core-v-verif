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
** Tests cv.elw, which exists only with COREV_CLUSTER. The testbench keeps
** pulp_clock_en_i at 1, so the clock is never gated and cv.elw reads the
** memory word like lw. The test runs cv.elw in a loop, with an immediate
** offset, twice in a row, and followed by an instruction that uses its result.
** Without COREV_CLUSTER cv.elw is illegal, as custom_opcode_illegal_test
** checks, and this test has nothing to do.
**
*******************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>

#ifdef CLUSTER

static volatile unsigned int data[4] = {0x11223344, 0xdeadbeef, 0x0, 0x5a5a5a5a};

static int check(const char *what, unsigned int got, unsigned int expected)
{
    if (got != expected) {
        printf("%s: read 0x%08x, expected 0x%08x\n", what, got, expected);
        return 1;
    }
    return 0;
}

int main(void)
{
    unsigned int v, w, sum = 0;
    int err = 0;

    for (int i = 0; i < 4; i++) {
        __asm__ volatile ("cv.elw %0, 0(%1)" : "=r"(v) : "r"(&data[i]) : "memory");
        err += check("cv.elw loop", v, data[i]);
        sum += v;
    }

    __asm__ volatile ("cv.elw %0, 12(%1)" : "=r"(v) : "r"(&data[0]) : "memory");
    err += check("cv.elw offset", v, data[3]);

    __asm__ volatile ("cv.elw %0, 4(%2)\n\t"
                      "cv.elw %1, 8(%2)"
                      : "=&r"(v), "=r"(w) : "r"(&data[0]) : "memory");
    err += check("cv.elw first of two", v, data[1]);
    err += check("cv.elw second of two", w, data[2]);

    __asm__ volatile ("cv.elw %0, 0(%1)\n\t"
                      "add %0, %0, %0"
                      : "=&r"(v) : "r"(&data[1]) : "memory");
    err += check("cv.elw then use", v, data[1] + data[1]);

    printf("cv.elw: sum 0x%08x, %d errors\n", sum, err);
    return err ? EXIT_FAILURE : EXIT_SUCCESS;
}

#else

int main(void)
{
    printf("cv.elw needs COREV_CLUSTER: nothing to check on this configuration\n");
    return EXIT_SUCCESS;
}

#endif
