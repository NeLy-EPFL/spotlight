# Code style and conventions

Conventions for contributing to this repository. The authoritative C++ formatter is
`.clang-format`; the points below cover what a formatter can't enforce.

## Language and build

- Use **C++20** for `trigger_firmware` and `comm_protocol` (the highest standard
  supported by `arduino-esp32 3.x` by default). Use **C++23** for `recorder`.
- Build with **CMake** (`recorder`, `comm_protocol`) and **PlatformIO**
  (`trigger_firmware`). `recorder` and `trigger_firmware` compile on their own;
  `comm_protocol` is a library included by both but can also be configured
  standalone to build and run its unit tests. See
  [Building and installing](setup/building.md).

## Formatting and naming

- Use the style specified in `.clang-format` and `.clang-tidy`.

  **clang-format** (formatting): run periodically on all tracked source files from the repo root:
  ```
  git ls-files '*.cc' '*.h' | xargs clang-format -i
  ```

  **clang-tidy** (naming and code quality): requires a built `recorder/build` (for `compile_commands.json`; this also covers `comm_protocol`). This should take about a minute. Most warnings are probably related to external libraries and are already supressed; these are safe to ignore. Check:
  ```
  run-clang-tidy -quiet -p recorder/build '.*spotlight-control/(recorder|comm_protocol)/(src|tests)/.*'
  ```
  Apply fixes automatically:
  ```
  run-clang-tidy -quiet -fix -p recorder/build '.*spotlight-control/(recorder|comm_protocol)/(src|tests)/.*'
  ```
- Use snake_case for file names, function names, variable names, and enum values, with `.cc` and `.h` suffixes.
- Use PascalCase for class, struct, enum, and type alias names (e.g. `RecorderConfig`, `FrameData`).
- Private member variables end with a trailing underscore (e.g. `image_width_`). Public members and all functions have no suffix.
- Namespaces are lowercase (e.g. `namespace config`).
- Use Google style for documentation in header files: `//` comment lines, not `/* */` blocks.
- When labelling arguments at a call site (e.g. for boolean flags or otherwise ambiguous literals), use the `/*name=*/value` style rather than a trailing `// name` comment:
  ```cpp
  // preferred (use new lines if needed)
  MuscleTriggerTiming timing(behavior_fps, /*sync_ratio=*/1, /*light_on=*/5000);

  // avoid
  MuscleTriggerTiming timing(behavior_fps,
      1,    // sync_ratio
      5000  // light_on
  );
  ```
- Keep documentation complete but concise. Assume the user has a certain level of technical know-how. Don't write docs just for the sake of it; make sure it's meaningful.
- Use `#pragma once` instead of `#ifndef` guards in header files.
- Use only ASCII characters except in `.md` files.

## Code quality

- Follow best practices in using `const` and pass arguments by reference when
  applicable.
- Write comments whenever the logic is unclear/non-trivial. Don't hard-code "magic
  numbers" or "magic logic."
- Mind overhead: prefer low overhead for `trigger_firmware` and `comm_protocol`
  (and `comm_protocol` must avoid exceptions, to respect the linear workflow on the
  microcontroller). For `recorder`, be mindful of overhead since frame acquisition
  can run at up to ~500 FPS — but don't over-optimize at the cost of readability.
- Write _meaningful_ unit tests only. Don't write tests just for the sake of it, and
  don't bloat test files to test trivial things.
