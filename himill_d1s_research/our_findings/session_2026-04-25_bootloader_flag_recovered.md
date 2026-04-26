# Session follow-up: bootloader entry flag recovered from vendor binaries

Date: 2026-04-25

## Outcome

The vendor's "specific flag" required to enter the HiMill bootloader from the
running application was recovered and verified on hardware.

`[BOOT]` now works.

## The flag

- Address: `0x2000BFF0`
- Value: `0xFA0505F5`

This flag must be written to SRAM immediately before requesting a system reset.

## Working sequence

The sequence that matches the vendor app is:

```c
*(volatile uint32_t *)0x2000BFF0 = 0xFA0505F5;
delay_about_10ms();

__set_FAULTMASK(1);
__DSB();
SCB->AIRCR = (SCB->AIRCR & 0x00000700U) | 0x05FA0004U;
__DSB();
for (;;)
    ;
```

Important details:

- Do not direct-jump into the bootloader for this path.
- Do not rely on a plain warm jump.
- Do not emit extra output right before reset if timing is tight.
- The SRAM flag must survive until the bootloader reads it very early after reset.

## Where this was patched

Implemented in:

- `boards/himill_d1s.c`
- `STM32F103RCTX_HIMILL_D1S_FLASH.ld`

The `[BOOT]` handler now uses the SRAM magic + reset sequence instead of the
old message+reset path.

The HiMill linker script reserves the top 16 SRAM bytes so the normal app stack
cannot overwrite the bootloader handoff word at `0x2000BFF0`. The shared
`STM32F103RCTX_FLASH.ld` is intentionally left generic for non-HiMill boards.

## How the flag was found

This was not guessed. It came from correlating the bootloader and vendor app.

### Step 1: find the bootloader's app-jump decision

The bootloader disassembly already existed:

- `bootloader/bootloader.bin`
- `bootloader/bootloader.asm`

The first useful target was not "find a magic flag" directly, but:

"Find where the bootloader decides whether to stay in bootloader mode or jump
to the application."

That led to the early startup code around:

- `bootloader.asm:0800073c`
- `bootloader.asm:080003bc4`

At `0x08003BC4`, the bootloader contains the app jump stub:

- read application SP from app base
- verify it points into SRAM (`0x20000000` range)
- load reset vector
- set MSP
- branch to app reset handler

So that confirmed the bootloader had a normal "jump to app" path and gave the
place to work backward from.

### Step 2: inspect the startup logic before that jump

The startup block at `0x0800073c` does this, in effect:

1. Load a word from SRAM
2. Compare it to a constant
3. If it matches:
   - clear a status byte / flag
   - run bootloader-side init
   - stay in a wait loop
4. Otherwise:
   - validate app marker
   - jump to `0x08008000`

The important literals decoded from the bootloader were:

- SRAM address `0x2000BFF0`
- magic value `0xFA0505F5`
- app-valid marker `0xAA55AA55`
- app base `0x08008000`

This was the first strong candidate for the vendor's "specific flag".

### Step 3: verify the candidate in the vendor app binary

A candidate found only in the bootloader could still be dead code or unrelated.
So the next step was:

"Search the vendor application binary for the same address and magic."

That immediately hit:

- `0x2000BFF0`
- `0xFA0505F5`

inside `HiMill_D1-v1.0.34-20251222.bin`, adjacent to each other.

That was the critical confirmation: the shipping app knows about the same
address/value pair as the bootloader.

### Step 4: disassemble around the literal in the vendor app

After locating the literals in the vendor app, disassembly around that region
showed the exact handoff sequence:

1. write magic to `0x2000BFF0`
2. delay about 10 ms
3. mask faults
4. issue `SYSRESETREQ` through `SCB->AIRCR`
5. loop forever waiting for reset

That removed the remaining ambiguity. It was no longer just "probable" - it
showed exactly how MaxMake does it.

## Why earlier attempts failed

Earlier attempts used either:

- direct jump into bootloader reset vector, or
- plain `NVIC_SystemReset()` without the SRAM flag

Those miss the bootloader's intended entry condition.

The bootloader was designed to distinguish:

- ordinary reset -> likely continue to app
- reset with SRAM magic present -> stay in bootloader

Without that flag, the bootloader never stays in the USB handshake window long
enough for the flasher path.

## Why this reverse-engineering approach worked

The useful pattern was:

1. Do not start by blind-searching for "flag" strings.
2. First locate the **state transition point**:
   bootloader-or-app decision.
3. Decode the literals participating in that decision.
4. Confirm those literals exist in the vendor app too.
5. Then disassemble around the vendor app hit to recover the full write/reset
   sequence.

That is the part worth reusing on future devices. The winning move was not
"more searching", it was choosing the right control-flow choke point first.

## Commands / techniques that helped

- `strings -a -t x ...`
  - quick way to see whether anything obvious exists at all
- `arm-none-eabi-objdump -D -b binary -m arm -Mforce-thumb,reg-names-std --adjust-vma=...`
  - decode raw binaries without needing ELF symbols
- `arm-none-eabi-objdump -s ...`
  - inspect literal pools near code
- direct literal search in the raw binaries
  - search for the candidate address and magic value in both bootloader and app

## Verified state now

- `[BOOT]` works on hardware
- bootloader flag and sequence are understood
- this item is no longer blocked on vendor reply

## Follow-up recommendation

Update the main `himill_d1s_research/README.md` so future sessions no longer
list bootloader entry as an unknown flag problem.
