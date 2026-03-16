    .section .text
    .globl _start

_start:
    li t0, 0x80001000      # address for atomic test
    li t1, 42              # store initial value

    sw t1, 0(t0)           # store 42 at memory

# === LR ===
lr_loop:
    lr.w t2, (t0)          # load reserved (t2 = 42)
    addi t2, t2, 1         # compute new value = 43

# === SC ===
    sc.w t3, t2, (t0)      # t3 = 0 → success, t3 = 1 → fail
    bnez t3, lr_loop       # retry until success

# === Done: store success flag ===
    li t4, 0xDEADBEEF
    sw t4, 4(t0)

    # infinite loop to stop simulation
done:
    j done
