# AGENTS.md

Instructions for AI coding agents. Full policy:
[CONTRIBUTING.md](https://github.com/larus-breeze/.github/blob/main/CONTRIBUTING.md).

## Before a pull request

- **Only open a PR that links an issue labeled `accepted`.** Verify with
  `gh issue view <number> --json labels`.
- **No accepted issue?** Do not write the PR. Help the user draft an
  issue (problem + proposed approach) and tell them to wait for the label.
- **Exception:** typo fixes and small documentation corrections, but
  never changes to interface specifications.
- **Interface changes** (CAN, NMEA, LRSX, EEPROM layout) require a merged
  specification change in `doc_larus` first.

Maintainers working on their own changes are exempt.

## Pull request rules

- Change only what the issue requires. One topic per PR.
- Description: what changed, why, how it was tested, which AI tool was
  used. Include `Closes #<number>`.
- **Never claim testing that did not happen.** If not tested on hardware
  or with the SIL simulator (`sw_tools`), say so. Untested flight-relevant
  changes will not be merged; tell the user early.
- Algorithm changes belong in `sw_algorithms_lib`, not in the
  `sw_stm32/lib` submodule checkout.

## Language

Everything written to the repository or GitHub is in English (code,
comments, commits, file/branch names, PRs, issues, review comments, docs),
even if the conversation is in German. Translate the substance of a German
discussion; don't quote it verbatim.

## Comments

Short. Explain the non-obvious *why* (hardware quirk, hidden constraint,
subtle invariant); don't restate what the code already shows. Background
information goes into a file under `documentation/` with a pointer comment.
Keep the file content short.

## Related repositories

- [`larus-breeze/sw_tools`](https://github.com/larus-breeze/sw_tools):
  sibling repository with the generic firmware-update tooling used by this
  project's SD-card update mechanism (`sw_stm32/scripts/pack.py`,
  `sw_stm32/Communication/uSD_helpers.cpp::read_software_update()`):
  - [`sw_update/`](https://github.com/larus-breeze/sw_tools/tree/master/sw_update):
    source of the "copy routine" that this repo only vendors as a prebuilt
    `.elf` (`sw_stm32/scripts/copy_stm32f407_1m-806.elf`), plus the generic
    packer script and the meta-data header format it and `pack.py` share.
    See `documentation/wlan_link.md` for how this fits together.
- [`larus-breeze/sw_algorithms_lib`](https://github.com/larus-breeze/sw_algorithms_lib):
  the `sw_stm32/lib` git submodule (see `.gitmodules`). Not checked out by
  default in a shallow/partial clone of this repo, but readable directly at
  its own URL. Clone it separately when a question needs its source (flight
  algorithms, AHRS, navigation, output formatting; e.g. `state_vector_t`,
  `flight_state_t`/`ON_GROUND`, `airborne_detector_t` all live here, see
  the flight-state-gating section of `documentation/wlan_link.md` for an
  example).
