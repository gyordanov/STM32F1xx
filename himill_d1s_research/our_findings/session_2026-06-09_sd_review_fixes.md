# D1S SD work review + fixes (2026-06-09)

## Context

Fresh-eyes embedded review of the ESP3D SD-cooperation work, commits
9369021 (start of the cooperation protocol) through ec40a33 (the SD read
fix), including the uncommitted submodule worktrees mirrored in
`himill_d1s_research/*_d1s_submodule_current.patch`.

Root-cause recap (verified in git, matches codex/opencode session recall,
mempalace drawer `6f9f93bb9e02`): commit 0a6a58e "defensively" rewrote
`xmit_spi()`/`rcvr_spi()` in `Src/diskio.c` — pre-write TXE waits plus a
bare `DR` read instead of `__HAL_SPI_CLEAR_OVRFLAG()`. On STM32F1 the OVR
flag only clears via DR-read-then-SR-read (the HAL macro does both), so the
"cleanup" could latch an overrun and occasionally drop exactly one byte
per SD sector, corrupting G-code near 512-byte boundaries while the files
on card were clean. ec40a33 reverted the helpers to the original timing.
Do not modernize those helpers again without a verifier build.

## Review verdict

The ownership protocol, handover re-init discipline (`power_off()` →
`CardType=0` + `STA_NOINIT`, SPI pins tri-stated on release), the
`xmit_datablock` NCR response polling, the survive-disconnect stream
rewrite, and the root-fs mount refcount fix all hold up. Four issues were
found and fixed this session.

## Fixes applied (plugin bumped to v0.10)

1. **Dead read-error abort path** — `sdcard/fs_stream.c`. `file.read_error`
   was never set (the setter was lost when the 240-byte read-ahead buffer
   was removed), so `stream_abort_on_read_error()` was unreachable and a
   mid-job disk error looked like EOF: silent job truncation, "Pgm End",
   spindle left running. Now `file_read()` clears `vfs_errno` before
   `vfs_read()` and sets `file.read_error = vfs_errno != 0` on a short
   read (FatFs EOF leaves FR_OK; a failing medium does not). The abort
   path additionally fires `protocol_enqueue_realtime_command(CMD_STOP)`,
   which discards the partial line accumulated in the protocol loop —
   without that, a truncated motion word (e.g. `Y2` left from `Y20`) would
   execute as valid g-code — and `stream_read()` returns `SERIAL_NO_DATA`
   instead of flushing the partial line with a synthetic EOL.
   Bench test: pull the SD card mid-job (no detect pin on D1S SPI socket);
   expect `error:29 in SD file at line N`, motion cancel, spindle off.

2. **`[D1S_SD_REQ]` honored mid-upload** — `boards/himill_d1s.c`. ESP3D
   (or any UART2 client) could take the bus while a `[D1SU:...]` upload had
   its file open, unmounting FatFs under it and stranding a partial file.
   The handler now answers `[D1S_SD_BUSY:upload]`. ESP3D forks that only
   prefix-match `[D1S_SD_BUSY` need no change.

3. **Debug-dump stack overflow** — `sdcard/fs_stream.c`
   `d1s_sddbg_verify_dump()` (only compiled with `D1S_SD_READ_VERIFY_DETAIL`).
   The trailing `']' + LF + NUL` writes were unguarded; with the 32-byte
   READ1/READ2 context dumps the NUL landed one past `char buf[112]`, and
   the `out += snprintf(...)` idiom could overshoot further on truncation.
   Rewritten with a reserved-tail bound (`end = buf + sizeof(buf) - 3`),
   clamped snprintf return, and buffer enlarged to 128 so a full 32-byte
   context now fits untruncated. Matters because this is the fallback
   debug build we'd flash for the next corruption hunt.

4. **Patch mirrors re-synced** — `grbl_d1s_submodule_current.patch` was
   stale: the grbl worktree had an unrecorded debug-only
   `d1s_gcdbg_report()` in `protocol.c` (logs the offending line text when
   `gc_execute_block` rejects it, gated by `D1S_SDDBG_ENABLE`). grbl and
   sdcard mirrors regenerated; keypad verified unchanged. Regeneration
   command: `command git -C <submodule> --no-pager diff >
   himill_d1s_research/<submodule>_d1s_submodule_current.patch`
   (plain `git` may be wrapped by rtk and reformat large diffs).

## Documented constraints (no code change)

- **`[D1S_SD_TAKE]` window**: while grblHAL waits (≤1.5 s) for
  `[D1S_SD_ACK]`/`[D1S_SD_BUSY]`, it drains UART2 wholesale. Realtime
  bytes (`!`, `?`, `0x18`) are safe — `mux_uart_enqueue_rt` intercepts
  them at enqueue time — but ordinary command lines forwarded by ESP3D in
  that window are dropped without error. Comment added at the drain loop
  in `himill_sd_reclaim_from_esp3d()`.
  **Implemented on the ESP3D fork the same day** (repo
  `/home/galin/Downloads/do_not_backup/cnc/ESP3D`, commit 34671730,
  v3.1.1-d7): `ESP3DSerialService::dispatch()` parks client→serial
  messages in a new `_messagesOnHold` FIFO (max 16, oldest dropped) while
  `ESP_SD::isD1SForwardHoldActive()` — true from `[D1S_SD_TAKE]` reception
  until the ACK/BUSY reply is written, self-expiring after 2 s; realtime
  bytes pass through. `ESP3DSerialService::handle()` flushes the parked
  messages once the hold clears. TAKE replies now carry a leading newline
  so a half-forwarded client line cannot glue onto the reply and make the
  STM32 drain miss it (the drain skips empty lines). Verified:
  `pio run -e esp32dev` SUCCESS, zero warnings, RAM 23.7 % / flash 72.2 %.
- **`$12` arc tolerance 0.005**: deliberate — coarser arc faceting means
  fewer, longer segments per arc so the reduced 50-block planner buffer
  stays fed; 5 µm chord error is inside machine accuracy. Rationale now in
  `platformio.ini`.

## Build verification

- Release config: SUCCESS, RAM 57.2 % (28,128 B), flash 85.7 % (224,604 B),
  IRQ vector post-check OK, zero warnings. Binary verified to contain
  `v0.10` and `[D1S_SD_BUSY:upload]`.
- Fallback debug config (`NGC_EXPRESSIONS_ENABLE=0`, `D1S_SDDBG_ENABLE=1`,
  `D1S_SD_READ_VERIFY=1`, `D1S_SD_READ_VERIFY_DETAIL=1`): SUCCESS,
  RAM 56.8 %, flash 81.2 %, zero warnings. Flags reverted after the check.

## Known risks left open (accepted)

- Upload bursts vs 512 B RX ring: 8 unacked ~360 B frames (~2.9 KB) can
  overflow RX if a slow FAT cluster write stalls the protocol loop;
  detected by seq/CRC → upload aborts and restarts. If field uploads abort
  on worn cards, shrink the ESP3D-side ack window or bump RX_BUFFER_SIZE.
- Sustained bulk output to UART2 (large `[D1SP:L]` listings, `$F<=` dumps)
  still has no downstream flow control — see
  `session_2026-05-28_sd_disconnect_and_bulk_output.md`.
