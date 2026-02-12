# AGENTS.md - sim-algo

Micromouse maze-solving algorithms using flood fill, tested via the
[MMS simulator](https://github.com/mackorone/mms) by mackorone.

## Priorities

1. Correctness -- algorithms must solve any valid 16x16 micromouse maze
2. Maintainability -- clean C99, no warnings with `-Wall -Wextra -Werror -pedantic`
3. MMS compliance -- stdout is reserved for simulator commands; all debug output goes to stderr

## Key Files

| file | purpose |
|------|---------|
| `src/api.h` | MMS simulator C API (single-header, included by each version) |
| `src/log.h` | Conditional logging API (LOG, LOG_ERROR, LOG_PHASE, LOG_STEP) |
| `src/ffv0.c` | Clean room: ffv3 architecture + enhanced log.h logging |
| `src/ffv1.c` | Baseline: search-only flood fill, stops at goal |
| `src/ffv2.c` | Full algorithm: search + return + speed run, exploration bonus |
| `src/ffv3.c` | Refactored v2: structs (MouseState, Maze, Point), direction enum, multi-goal BFS |
| `makefile` | Build system (`make all`, `make debug`, `make clean`) |
| `docs/design.md` | Algorithm design notes and version progression |
| `docs/style-guide.md` | C coding standards for this project |

## Build

```sh
make all              # release build with logging enabled by default (-O2 -DLOG_ENABLED)
make LOG_ENABLED=false all  # release build without logging (-O2)
make ffv0             # build only ffv0.out (with logging)
make ffv1             # build only ffv1.out (with logging)
make ffv2             # build only ffv2.out (with logging)
make ffv3             # build only ffv3.out (with logging)
make debug            # debug build with logging (-O0 -g -DDEBUG -DLOG_ENABLED)
make clean            # remove artifacts
```

## Testing

Run executables in the MMS simulator. No unit test framework -- correctness
is verified by successful maze completion across different maze configurations.

1. Open MMS simulator
2. Configure mouse: directory = project root, build = `make ffv3`, run = `./ffv3.out`
3. Run with multiple maze files to verify

## Architecture Notes

- Each version is a standalone .c file that includes api.h
- 16x16 grid, all data is stack-allocated (no malloc)
- BFS-based flood fill with multi-source seeding for 2x2 center goal
- State machine: SEARCH_MODE -> RETURN_MODE -> SPEED_MODE (v2/v3/v0)
- Walls are permanent once detected (never cleared)

## Style

- C99 strict, 2-space indent, snake_case, see `docs/style-guide.md`
- Use `snprintf` over `sprintf`, check `fgets` return values
- All logging via `fprintf(stderr, ...)`, never `printf` for debug
