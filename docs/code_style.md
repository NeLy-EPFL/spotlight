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

- Use the style specified in `.clang-format`. Don't use `clang-tidy`.
- Use snake_case for file names, with `.cc` and `.h` suffixes.
- Use camelCase for variable names. For abbreviations, use all caps instead of capitalizing only the first letter (e.g. `blueLED` instead of `blueLed`). If the abbreviation is the first word, keep lower case (e.g. `ledPin`). Exception: for ID and IDX, use `Id` or `Idx`.
- Private attribute names should end with an underscore, but not private functions.
- Use `/**` for docstrings and `//` for comments.
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
