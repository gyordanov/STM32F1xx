# Session 2026-05-10 to 2026-05-11 - D1S stream/RGB/toolchange hardening

This note captures the bugs and fixes from the May 10-11 D1S testing
cycle. Read this before touching MPG, RGB, probing, NVS writes, or the
D1S toolchange path again.

## Final result

The D1S branch was updated as a single commit over `gh/d1s`:

```text
41c39cb d1s: harden toolchange and stream stability
```

That commit intentionally keeps grblHAL core edits as patch files under
`himill_d1s_research/` instead of committing a new `grbl` submodule SHA:

- `grbl_no_settings_descriptions.patch`
- `grbl_nvs_sync_idle_delay.patch`
- `grbl_toolchange_restore_soft_limits.patch`

The local firmware test build used the patched grblHAL core plus the
D1S board changes. If a clean checkout is used, apply the patch files to
the `grbl` submodule before expecting the exact tested behavior.

## 1. MPG stop / undefined feed rate after power-cycle

### Symptoms

Observed on D1S with SmartPendant on UART4, not reproduced on FlexiHAL:

- Pendant sometimes showed `Gcode Undefined Feed Rate`.
- In another run the pendant/MPG simply stopped jogging.
- It was easiest to trigger soon after a power-cycle, especially with
  repeated Z jogging.
- Temporary `[MPGLOG]` output showed mostly valid `$J=` lines, followed
  by raw corruption / repeated `0x8B` bytes:

```text
[MPGLINE:$J=G21G91Z-0.500F600.00]
...
[MPGRAW:...$J=G21G91Z-0.500$J=G21G91Z-0.500F600.00\r\x8B\x87\x8B...]
```

### Working theory

The D1S WS2812 status LED driver bit-bangs with interrupts disabled.
That is normally a short block, but it can still land while UART4 is
receiving pendant bytes or while the ESP/USB stream is receiving `$J=`
jog bursts. Losing or merging bytes in a jog line explains undefined
feed-rate errors and the corrupted raw log.

This is a D1S-specific risk because the RGB implementation and stream
mux are board-specific. FlexiHAL did not reproduce the failure.

### Fix

RGB writes are now runtime-gated and jog-safe:

- `$452=0` disables all D1S WS2812 writes. This is the default.
- `$452=1` enables automatic state colors and `M150`.
- Automatic RGB updates are deferred during jogging, regardless of
  whether the jog source is MPG or the sender.
- The idle color is applied after jogging has been quiet briefly.

Temporary `[MPGLOG]` / `rgbdefers` instrumentation proved the concept
and was removed once the tester confirmed the fix.

### Do not regress

Do not add LED animations or frequent RGB refreshes until UART receive
loss has been re-tested under:

- MPG Z jogging on UART4
- sender-controlled `$J=` jogging
- SD-card job plus sender connection
- web UI / websocket connection active

The safe default for production D1S builds is no RGB writes unless the
user explicitly enables them with `$452=1`.

## 2. D1S profile repair on boot

The D1S firmware is machine-specific, not just controller-specific. Bad
machine settings can crash tools or skip the toolsetter. After one test
machine booted with unsafe values, the board code was made more forceful.

On boot, D1S repairs critical settings and reports:

```text
[MSG:D1S repaired]
```

Forced values:

```text
$10=6143
$650=3
$20=1
$21=1
$40=1
$65=10
$341=3
$342=65
$346=7
```

`$22` is handled specially. The board forces only the "set machine
origin to 0" homing bit, while leaving the other homing option bits
user-configurable.

## 3. Probe burst drops after `G10 L20`

### Symptoms

Only reproduced on D1S. FlexiHAL handled the same sender sequence.

Examples from sender debug traces:

- `/tmp/have-to-probe-twice-gusender-debug-2026-05-10_16-50-38.json`
- `/tmp/error-71-gusender-debug-2026-05-10_17-16-09.json`

The sender sent bursts such as:

```gcode
G10 L20 X0
G91 G21
G38.2 X30 F150
```

or:

```gcode
G10 L20 X0
G90
```

The D1S acknowledged only the first command. The work coordinate offset
changed, proving `G10 L20` executed, but `$G` still reported the old
modal state (`G91` when `G90` should have applied). The controller then
sat Idle with no alarm and no further `ok`, or later returned a confusing
`error:71` on the next motion attempt.

### Root cause

`G10 L20` dirties coordinate/NVS data. On STM32F1, the physical flash
erase/program done by `nvs_buffer_sync_physical()` can stall long enough
for the ESP/UART stream path to lose the immediately following line.

The bug is timing-sensitive and looks like "the firmware silently ignored
line 2 of the burst". It is not actually a sender-side attribution issue.

### Fix

Core patch:

```text
himill_d1s_research/grbl_nvs_sync_idle_delay.patch
```

D1S build flag:

```ini
-D NVS_SYNC_IDLE_DELAY_MS=500
```

The patch defers physical NVS sync until the stream has been quiet for
500 ms. It does not change when settings/coordinate data are logically
updated; it only avoids starting an STM32 flash write while the command
burst is still arriving.

## 4. Bootloader handshake failure that is really the app

This error usually means the D1S never entered the HiMill bootloader:

```text
Handshake failed: expected a10201 or a10401, got 3c4964
```

`3c4964` is ASCII for `<Id`, the start of a grblHAL status report such
as `<Idle|...>`. That means the flasher is talking to the running app,
not the bootloader.

Common cause seen during testing: an active sender/serial connection was
still attached, so boot mode did not take reliably.

`tools/himill_flash.py` now formats unexpected handshake bytes as both
hex and ASCII and prints a targeted hint when the reply looks like:

- grblHAL status (`<Idle|...>`)
- grblHAL startup banner
- normal `ok` / `error` output
- no reply at all

Windows `pyusb` without a usable libusb backend is also handled by a
pyserial DTR/RTS fallback after flashing, so a completed flash no longer
ends with an avoidable `usb.core.NoBackendError` traceback.

## 5. Physical button during toolchange

### Suspicious user report

During a multi-tool SD job, the machine reached the first toolchange.
After the user changed the bit and pressed the physical D1S button, the
machine reportedly resumed cutting instead of probing the toolsetter.
This was not reproduced locally, and the sender-side connection state was
unknown.

### Risk path

grblHAL's M6 path waits for `CMD_TOOL_ACK` before installing the
cycle-start behavior that runs the automatic touch-off. If the sender
disconnects before the ack is processed, or the physical PC4 button lands
before the ack path completes, a normal cycle-start can resume the job
without the toolsetter sequence.

### Fix

`boards/himill_d1s.c` now makes the physical PC4 button toolchange-aware:

- If in `STATE_TOOL_CHANGE` and `gc_state.tool_change` is active, the
  button first sends `CMD_TOOL_ACK` through the mux realtime handler or
  calls `grbl.on_toolchange_ack()`.
- Only after the ack path has installed the proper cycle-start trap does
  the normal cycle-start callback run.

This also works around sender disconnects during automatic touch-off at
G59.3 plus change-at-G30, which is the D1S-supported M6 workflow.

## 6. Toolchange restore soft-limit alarm at Z0

### Trace

`/tmp/soft-limits-gusender-debug-2026-05-10_21-18-24.json`

Pattern:

- SD job started with `$F=/nomodal_tc_test_2.nc`.
- Controller entered `<Tool ... T:1>`.
- Toolsetter sequence probed successfully.
- Final TLO was tiny but positive, for example `TLO:0.003`.
- Machine was near the top boundary (`MPos Z0`) with soft limits on.
- Restore after toolchange alarmed:

```text
Alarm 2: Soft limit - target exceeds travel
[MSG:Reset during streaming of file at line: 12]
```

### Root cause

The restore target was effectively "go back to exactly Z0". After TLO
math, a tiny positive offset could put the target just outside the legal
machine envelope. This is why the alarm happened after probing/retract,
not on a sender command.

### Fix

Core patch:

```text
himill_d1s_research/grbl_toolchange_restore_soft_limits.patch
```

The patch adjusts the saved restore target by the active linear offset
and then clamps it through `grbl.apply_travel_limits()` before issuing
the return move.

Starting a job slightly below machine Z0 or moving G30 Z to `-1` would
also avoid the specific boundary symptom, but the patch is the right fix.

## 7. Debugging checklist for next time

When a D1S-only stream/probe/toolchange bug shows up again, capture:

- Sender JSON debug trace.
- `$I` output, especially `[PLUGIN:HIMILL_D1S v0.3]` or later.
- `$$`, `$#`, `$G`, and current status report around the failure.
- Whether the run is USB, ESP websocket, ESP telnet, SD `$F`, or MPG.
- Whether `$452` is 0 or 1.
- Whether RGB/M150 commands are active.
- Whether the issue happens once after power-cycle or repeats in one
  session.
- Exact physical action timing for M6: sender modal visible, bit changed,
  button press, connection lost/reconnected, etc.

If MPG corruption comes back, re-add only a bounded UART4 ring-buffer
dump. Do not leave high-volume MPG debug output in the production build.
