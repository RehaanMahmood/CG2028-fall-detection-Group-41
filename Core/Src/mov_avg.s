/*
 * mov_avg.s
 *
 * CG2028 Assignment starter file.
 */
.syntax unified
.cpu cortex-m4
.thumb
.global ewma_filter
.type ewma_filter, %function

.text
.align 2

@ CG2028 Assignment
@ (c) ECE NUS
@ Write Student 1's Name here: Rehaan Mahmood (A0305908B)
@ Write Student 2's Name here: Fahad Mohaideen Abdul Kader (A0000007X)
@
@ Function prototype:
@   int ewma_filter(int new_data, int old_output, int alpha_percent);
@
@ ARM calling convention:
@   R0 = new_data       (signed integer sensor sample)
@   R1 = old_output     (previous filtered output)
@   R2 = alpha_percent  (integer from 0 to 100)
@   Return R0 = (alpha_percent * new_data
@                + (100 - alpha_percent) * old_output) / 100
@
@ Notes:
@ - Use signed integer arithmetic.
@ - Integer division must truncate towards zero, matching C integer division.
@ - Preserve all callee-saved registers that you use (R4-R11).
@ - Do not call a C helper function and do not use floating-point instructions.
@
@ Register table:
@   R0: new_data (input) / final filtered output (return value)
@   R1: old_output (input)
@   R2: alpha_percent (input), reused as divisor before SDIV
@   R3: (100 - alpha_percent)
@   R4: alpha_percent * new_data, then accumulates numerator
@   R5: (100 - alpha_percent) * old_output
@
@ Write your program from here.
ewma_filter:
    PUSH {r4, r5, lr}

    @ TODO: Implement the EWMA low-pass filter in pure ARM assembly.
    MOV   r3, #100
    SUB   r3, r3, r2        @ r3 = (100 - alpha_percent)
    MUL   r4, r2, r0        @ r4 = alpha_percent * new_data
    MUL   r5, r3, r1        @ r5 = (100 - alpha_percent) * old_output
    ADD   r4, r4, r5        @ r4 = numerator
    MOV   r2, #100
    SDIV  r0, r4, r2        @ r0 = numerator / 100, truncated toward zero

    POP  {r4, r5, pc}

.size ewma_filter, .-ewma_filter
