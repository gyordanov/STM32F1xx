# Session 2026-05-12 - D1S state, defaults, and resume notes

This is the current handoff state for the D1S firmware work as of
May 12, 2026.

## Branch state

Current local checkout during this note:

```text
branch: himill_d1s
HEAD:   5cd9913 oh and yes set spindle delay
```

Pushable D1S branch:

```text
d1s tip: one local commit named "d1s: harden toolchange and stream stability"
base:    41e61a3 d1s: stabilize RGB jogging and repair settings
remote:  gh/d1s was previously seen at an older pre-memory version of the tip
```

If preserving the one-commit D1S branch shape, push with:

```bash
git push --force-with-lease gh d1s
```

Do not merge the whole `himill_d1s` branch into `d1s`. It contains old
bring-up history and unrelated submodule/research drift. The intended
`d1s` branch shape is one squashed commit over the previous D1S commit.

## Dirty local state to remember

The top-level working tree had the `grbl` submodule dirty during the
latest tested builds. The important grblHAL core changes are stored as
patch files instead of a new submodule commit:

```text
himill_d1s_research/grbl_no_settings_descriptions.patch
himill_d1s_research/grbl_nvs_sync_idle_delay.patch
himill_d1s_research/grbl_toolchange_restore_soft_limits.patch
```

Reason: `grbl` points at upstream `grblHAL/core`. Committing a local
submodule SHA that is not available from an accessible remote would make
clean clones brittle. For a clean checkout, apply the patches to the
`grbl` submodule before expecting the exact tested D1S behavior.

The current local submodule edits were:

```text
grbl/settings.c      - NO_SETTINGS_DESCRIPTIONS flash-size patch
grbl/protocol.c      - NVS physical sync idle delay
grbl/tool_change.c   - restore target soft-limit clamp after TLO math
```

## Current D1S stability fixes

Read `session_2026-05-10_to_11_stream_rgb_toolchange.md` for full
details. Short version:

- RGB is disabled by default with `$452=0`.
- `$452=1` enables state colors and `M150`.
- Automatic RGB writes are deferred during jogging from any source.
- Boot-time D1S profile repair forces critical machine settings and
  reports `[MSG:D1S repaired]` if it changes anything.
- `NVS_SYNC_IDLE_DELAY_MS=500` prevents `G10 L20` from triggering an
  STM32F1 flash write while the next line of a probe burst is arriving.
- Physical PC4 during M6 first runs the grblHAL toolchange ack path
  before cycle-start can resume.
- Toolchange restore is clamped through soft limits so tiny positive
  TLO values do not make a restore-to-Z0 target exceed travel.
- The flasher now decodes bad bootloader handshakes. `3c4964` is
  ASCII `<Id`, meaning the app is still running and the user should
  close active senders/serial monitors, power-cycle, and retry.

## Current D1S acceleration defaults

Keep these as the public defaults:

```gcode
$120=500 ; X acceleration, mm/sec^2
$121=500 ; Y acceleration, mm/sec^2
$122=250 ; Z acceleration, mm/sec^2
```

Rationale:

- X/Y max rate is `2000 mm/min` (`33.3 mm/sec`). At `500 mm/sec^2`,
  the axis reaches max speed in about `0.067 s` over about `1.1 mm`.
- Z max rate is `1500 mm/min` (`25 mm/sec`). At `250 mm/sec^2`, Z
  reaches max speed in about `0.1 s` over about `1.25 mm`.
- `$451=1` SpreadCycle is the default, so these accel values have the
  torque response needed for cutting and jogging.
- Higher defaults mainly make jogging and tiny segments snappier, but
  increase vibration / missed-step risk on a small machine with
  standalone TMCs.

If a very conservative fallback profile is needed for a problem user:

```gcode
$120=300
$121=300
$122=150
```

Do not make the conservative values the firmware default unless field
testing shows the current defaults are unreliable.

## Critical D1S settings

Boot-time repair currently forces:

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

`$22` is special: only the "set machine origin to 0" homing bit is
forced. Other homing option bits stay user-configurable.

Known important defaults from `platformio.ini`:

```text
$100/$101/$102=1600
$103=88.889
$110/$111=2000
$112=1500
$113=2000
$120/$121=500
$122/$123=250
$130=280
$131=200
$132=80
$451=1  SpreadCycle
$452=0  RGB disabled
```

## Next likely tasks

- Push `d1s` after reviewing the single squashed commit.
- Decide whether to keep grblHAL core fixes as D1S patch files forever
  or turn them into upstreamable PRs.
- If upstreaming, start with the NVS sync idle delay and toolchange
  restore clamp. The D1S physical-button ack path is board-specific.
- Keep RGB animations off the table until worst-case MPG and sender
  jogging have been tested with `$452=1`.
- If a probing/toolchange bug appears again, capture sender JSON trace,
  `$I`, `$$`, `$#`, `$G`, status reports around failure, transport
  path, `$452`, and exact M6 physical button timing.
