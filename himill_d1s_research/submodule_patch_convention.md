# D1S submodule patch convention

D1S firmware uses upstream-ish grblHAL submodule commits plus top-level patch files.

## Release model

- Keep parent `STM32F1xx` branch pointing submodules at base commits that GitHub users can fetch.
- Keep D1S-only submodule changes as patch files in this parent repo.
- Squash-merge parent work to the D1S release branch when ready.
- Builders should be able to clone/pull parent repo, initialize submodules, apply patches, then build firmware.

## Apply patches

Run from repo root:

```bash
git submodule update --init --recursive
bin/apply-d1s-submodule-patches
```

The script applies only the cumulative patch files:

- `himill_d1s_research/grbl_d1s_submodule_current.patch`
- `himill_d1s_research/keypad_d1s_submodule_current.patch`
- `himill_d1s_research/sdcard_d1s_submodule_current.patch`

Do **not** blindly apply every `*.patch` in `himill_d1s_research/`. That folder also contains ingredient/research patches used to explain or recreate parts of the cumulative patches.

## Updating patches during development

Local development may use detached submodule commits for convenience. Before release, regenerate cumulative patches from the parent gitlink base to the tested local submodule state:

```bash
git -C grbl diff <parent-grbl-gitlink>..HEAD > himill_d1s_research/grbl_d1s_submodule_current.patch
git -C keypad diff <parent-keypad-gitlink>..HEAD > himill_d1s_research/keypad_d1s_submodule_current.patch
git -C sdcard diff <parent-sdcard-gitlink>..HEAD > himill_d1s_research/sdcard_d1s_submodule_current.patch
```

Then make sure the parent commit tracks:

- board/build changes (`boards/himill_d1s.c`, `platformio.ini`, etc.)
- updated cumulative patch files
- base submodule gitlinks, not private detached submodule commits

## Current v0.13 base intent

- Parent gitlinks stay at fetchable base commits.
- D1S `$453` MPG gate and related grbl/keypad/sdcard changes live in cumulative patches.
- Local build machine may keep submodule worktrees already patched/committed; parent release must not depend on those private commits.
