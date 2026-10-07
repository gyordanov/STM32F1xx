# D1S SD disconnect policy and bulk-output mux notes (2026-05-28)

## Context

This note records follow-up from EGSender/D1S testing around SD-card jobs,
manual SD file dumps, and the STM32F1xx D1S stream mux.

Relevant owned firmware tree:

- `/home/galin/Downloads/do_not_backup/cnc/STM32F1xx`
- D1S mux: `boards/himill_d1s.c`
- SD file stream commands: `sdcard/fs_stream.c`
- UART buffering: `Src/serial.c`
- D1S build flags: `platformio.ini` `[env:himill_d1s]`

The ESP3D firmware on the tested machine is stock MaxMake. Treat it as a
black box unless deliberately planning a separate ESP3D investigation.

## Design todo: SD jobs should survive sender/client disconnects

Desired D1S behavior:

- Once a job has been handed to the controller as an SD-card run (`$F=<path>`),
  it should continue if the UI/client disconnects.
- This should be true regardless of which client started it:
  - ESP3D websocket
  - ESP3D telnet
  - USB CDC
- A sender reconnect should observe/resync state rather than implicitly stop the
  SD job.

Rationale:

- SD runs are controller-owned once started.
- EGSender/user workflow expects the machine to keep running if the browser,
  websocket, telnet, or USB host session drops.
- Stop/hold/reset should remain explicit realtime/operator actions, not a side
  effect of UI transport loss.

Future work:

- Audit grblHAL `stream_changed` / file-stream disconnect behavior for D1S.
- Make D1S policy explicit in tests and comments: transport disconnect must not
  terminate an active SD-card job.
- Validate for both network and USB starts:
  1. Start SD job over websocket, disconnect UI, confirm job continues.
  2. Reconnect UI, confirm state/progress can be observed.
  3. Repeat over telnet if useful.
  4. Start SD job over USB CDC, close USB host session, confirm job continues.
  5. Confirm explicit feed-hold, cycle-start/resume, stop/reset still work.

## Manual `$F<=` bulk-output finding

Manual `$F<=<path>` dumps an SD file to the active output stream. This is not
used by normal EGSender SD-load flow on D1S: EGSender uses HTTP for SD file
loading/downloading in HTTP mode. Normal SD job execution uses `$F=<path>`, not
`$F<=<path>`.

Observed discriminator tests:

- `$F<=/d1s_rotation_smoke.nc` works over the D1S network path.
- `$F<=/b-top-cuts.nc` works over USB CDC.
- `$F<=/b-top-cuts.nc` fails over ESP3D websocket: no complete response lines,
  then websocket close code 1006.
- `$F<=/b-top-cuts.nc` also fails over telnet: connection closes with no file
  response.
- `$F` listing after reconnect still shows `/b-top-cuts.nc|SIZE:78173`, so the
  file exists and the SD plugin/list path is healthy.

Current interpretation:

- This is not a generic FatFs/file-open failure.
- This is not websocket-specific, because telnet also fails.
- This is likely sustained bulk-output/backpressure through the STM32F1xx D1S
  mux/UART2 -> stock MaxMake ESP3D path.
- It probably affects manual diagnostics/bulk dumps far more than regular jobs.

Why `$$` / `$ESH` passing is not enough:

- D1S build sets `TX_BUFFER_SIZE=8192` in `platformio.ini`.
- Existing comments say this was intentionally oversized to absorb settings
  enumeration bursts without blocking.
- In the debug log, approximate output sizes were:
  - `$$`: about 0.9 KiB
  - `$ESH`: about 8 KiB
  - `/b-top-cuts.nc`: 78,173 bytes / about 4,079 lines
- `$$` and `$ESH` likely fit inside, or barely inside, the 8 KiB STM32 TX ring.
- `$F<=/b-top-cuts.nc` necessarily enters the sustained blocking/drain path.

Relevant code:

- `sdcard/fs_stream.c` `cmd_to_output` reads the whole file synchronously and
  writes one character/newline at a time through `hal.stream.write`.
- `boards/himill_d1s.c` routes ordinary output to sticky `mux_reply_owner` via
  `mux_write_target()`.
- `boards/himill_d1s.c` treats UART2 as always connected because it is hardwired
  to ESP3D; STM32 cannot see whether stock ESP3D has an active/healthy network
  client or whether its network side is congested.
- `Src/serial.c` only backpressures on the STM32 UART TX ring filling. There is
  no downstream ESP3D/network flow-control signal.

## Regular-job risk assessment

Likely safe/low risk:

- Streamed jobs: `ok`-flow-controlled; not one huge one-way dump.
- SD `$F=<path>` jobs: controller reads SD locally and emits status/progress,
  not the whole file content.

Potentially risky canaries:

- Manual `$F<=<large-file>` dumps.
- Very large `$F` listings.
- Long diagnostic reports that exceed the 8 KiB TX buffer by a meaningful
  margin.
- Any future feature that produces sustained network-bound text output without
  sender-side flow control.

## Research/testing backlog

Threshold tests over ESP3D websocket and/or telnet:

1. `$F<=` file just below 8 KiB.
2. `$F<=` file just above 8 KiB, e.g. `/2026-01-17_10-42-37.nc` if still around
   8,940 bytes.
3. Medium file, e.g. `/1001.nc` if still around 27,600 bytes.
4. Large file, e.g. `/b-top-cuts.nc` at 78,173 bytes.

Instrumentation ideas:

- Count bytes emitted by `cmd_to_output` before failure/close.
- Track max UART2 TX buffer occupancy.
- Count calls to the serial blocking callback during bulk output.
- Count failed/aborted `serialPutC` returns.
- Expose a temporary `[MSG:D1S bulk ...]` or debug GPIO pulse before/after file
  open, first byte, every N bytes, and EOF.
- Compare STM32 UART2 bytes on a logic analyzer with bytes seen by ESP3D
  network client.

Candidate mitigations, safest first:

1. Sender/UI policy: do not use `$F<=` for large D1S network downloads; use HTTP
   or USB CDC. EGSender already uses HTTP for normal SD load in HTTP mode.
2. Firmware guard: reject or warn for large `$F<=` when the reply owner is UART2
   / ESP3D, suggesting HTTP/USB instead.
3. Add diagnostic counters first, before changing global mux behavior.
4. Targeted pacing only inside `cmd_to_output` / `$F<=` bulk dump path.
5. Avoid global UART/mux pacing unless tests show normal jobs need it; global
   pacing could affect streamed-job latency and real-time feel.

## Upstream grblHAL hint

A quick look at grblHAL core and common drivers did not show another direct
match for the D1S design where two ordinary command clients are multiplexed with
sticky reply ownership.

Useful upstream patterns:

- grblHAL normally has one active `hal.stream`; `write_all` broadcasts status /
  alarms to connected streams, while ordinary `hal.stream.write` goes to the
  active stream.
- MPG is an exclusive mode switch (`stream_mpg_enable()` saves current stream,
  swaps in MPG handlers, and restores later), not a concurrent command mux.
- `stream_passthru.c` forwards UART -> USB in bounded 64-byte chunks via a
  delayed task, which is a useful pattern for bridge-style forwarding/pacing.

Implication:

- The D1S mux is necessarily custom because stock MaxMake ESP3D is a raw bridge
  on UART2 and USB CDC must also work.
- Keep the custom mux, but be conservative about sustained bulk output and make
  SD-job disconnect semantics explicit and tested.
