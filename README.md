# Algorithm Simulations

This repository contains maze solving algorithms that are under simulation testing.
Currently we are using the [mms simulator](https://github.com/mackorone/mms) by mackorone for our software simulations of maze navigation.

## Getting Started

1. Have the [relevant mms simulator downloaded](https://github.com/mackorone/mms?tab=readme-ov-file#download) and running on your platform.
2. Clone this repo
3. Check if you have relevant c toolchain available for your platfrom to compile the C algorithm example.

> [!NOTE]
>
> You can try to install gcc or msvc if you are on Windows. Unix platforms generally include gcc.
> For an even easier setup on Windows try using [Zig](https://ziglang.org/download/) as a C compiler.

4. Build from the project root:

```sh
make all                # builds all versions (ffv0.out, ffv1.out, ffv2.out, ffv3.out) with logging enabled
make LOG_ENABLED=false  # builds without logging (for embedded deployment)
make ffv0               # builds only ffv0.out (recommended - clean room implementation)
make ffv1               # builds only ffv1.out
make ffv2               # builds only ffv2.out
make ffv3               # builds only ffv3.out
make debug              # builds with debug symbols (-g -DDEBUG)
make clean              # removes build artifacts

# Use a different compiler (e.g., zig, clang)
make CC="zig cc"
make CC=clang

# Disable logging for production/embedded
make LOG_ENABLED=false all
```

5. Click any of the two buttons next to Mouse in the Simulator and add the relevant algorithm you want to test.

![mms guide](https://files.catbox.moe/dnbnp5.png)

- Enter the project root directory in the Directory field
- Build command: `make ffv0` (recommended) or `make ffv3` or `make all`
- The Run Command should point to the executable: `./ffv0.out` or `./ffv3.out`

> [!TIP]
>
> **Using Zig as C compiler (recommended for Windows):**
>
> Download [Zig](https://ziglang.org/download/), add to PATH, then:
>
> ```sh
> make CC="zig cc" all
> ```
>
> Or set it in MMS build command: `make CC="zig cc" ffv3`

## Project Structure

```
sim-algo/
├── src/           # Source code
│   ├── api.h      # MMS simulator API interface in C
│   ├── log.h      # Conditional logging system (LOG, LOG_ERROR, LOG_PHASE, LOG_STEP)
│   ├── ffv0.c     # Clean room: ffv3 + enhanced logging (RECOMMENDED)
│   ├── ffv1.c     # Goal search only (baseline)
│   ├── ffv2.c     # Search + return + speed run
│   └── ffv3.c     # Refactored v2 with structs and enums
├── assets/        # Maze files and tools
│   ├── fetch_mazes.py  # Download mazes from tcp4me and micromouseonline
│   └── mazes/          # Downloaded maze files (gitignored)
├── docs/          # Documentation and assets
│   ├── design.md       # Algorithm design notes
│   └── style-guide.md  # C coding standards
├── makefile       # Build system
├── AGENTS.md      # Project guide for AI agents
├── license
└── readme.md      # This file
```
