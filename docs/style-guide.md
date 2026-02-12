## c style guide (sim-algo)

### standard and compilation
- use C99: `-std=c99 -Wall -Wextra -Werror -pedantic`
- treat all warnings as errors, no exceptions
- compile with `-O2` for release, `-O0 -g` for debug

### naming
- `snake_case` for functions, variables, files
- `UPPER_SNAKE_CASE` for constants and macros
- `PascalCase` for struct/enum type names
- prefix api functions with module name: `API_moveForward()`

### types
- use `<stdint.h>` fixed-width types: `int8_t`, `uint8_t`, `int16_t`, etc.
- use `<stdbool.h>` for boolean values
- use `int` only for loop counters and trivial arithmetic
- avoid `unsigned` for sizes unless bit manipulation is involved

### formatting
- 2-space indentation, no tabs
- braces on same line: `if (x) {`
- single blank line between functions, no other blank line runs
- max line length: 80 characters (enforced by clang-format)

### structs and data
- prefer structs over parallel arrays for related data
- initialize all variables at declaration
- use designated initializers: `(Point){.x = 0, .y = 1}`

### error handling and output
- all debug/log output goes to `stderr` via `fprintf(stderr, ...)`
- `stdout` is reserved for MMS simulator commands only
- never use `printf()` for debug output -- this breaks MMS communication

### memory
- all maze data is stack-allocated (fixed 16x16 grid)
- no dynamic allocation needed for this project
- if dynamic allocation is ever needed: check return, free in same scope

### file organization
- api layer in `api.h` (single header, no .c pair)
- each algorithm version is a standalone `.c` file that includes `api.h`
- no shared state between files
