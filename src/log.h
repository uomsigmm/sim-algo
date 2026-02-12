/// log.h -- conditional compilation logging for sim-algo
/// single-header, C99 strict, zero overhead when disabled
///
/// usage:
///   #include "log.h"
///   LOG("mouse at (%d,%d)", x, y);
///   LOG_ERROR("no valid path from (%d,%d)", x, y);
///   LOG_PHASE("SEARCH -> RETURN [step #%d, pos (%d,%d)]", step, x, y);
///   LOG_STEP(step, x, y, 'N');
///
/// enable:  compile with -DLOG_ENABLED
/// disable: compile without it (default -- zero binary overhead)
///
/// all output goes to stderr. stdout is reserved for MMS simulator commands.
/// format string checking works in both enabled and disabled modes.

#ifndef LOG_H
#define LOG_H

#include <stdio.h>

#ifdef LOG_ENABLED

#include <stdarg.h>

__attribute__((format(printf, 2, 3)))
static inline void log_trace(const char *tag, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  fputs(tag, stderr);
  vfprintf(stderr, fmt, args);
  fputc('\n', stderr);
  va_end(args);
}

__attribute__((format(printf, 4, 5)))
static inline void log_error(
    const char *tag, const char *file, int line,
    const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  fprintf(stderr, "%s%s:%d: ", tag, file, line);
  vfprintf(stderr, fmt, args);
  fputc('\n', stderr);
  va_end(args);
}

__attribute__((format(printf, 2, 3)))
static inline void log_phase(const char *separator, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  fputs(separator, stderr);
  vfprintf(stderr, fmt, args);
  fputs(" ", stderr);
  for (int i = 0; i < 30; i++) fputc('-', stderr);
  fputc('\n', stderr);
  va_end(args);
}

static inline void log_step(int step, int x, int y, char dir) {
  fprintf(stderr, "#%d (%d,%d) %c\n", step, x, y, dir);
}

/// general trace/info -- output: [LOG] <message>
#define LOG(...) log_trace("[LOG] ", __VA_ARGS__)

/// error with source location -- output: [ERR] file.c:42: <message>
#define LOG_ERROR(...) \
  log_error("[ERR] ", __FILE__, __LINE__, __VA_ARGS__)

/// phase transition banner -- output: ------------ <message> --------------
#define LOG_PHASE(...) log_phase("------------ ", __VA_ARGS__)

/// compact position trace -- output: #<step> (<x>,<y>) <dir>
#define LOG_STEP(step, x, y, dir) log_step((step), (x), (y), (dir))

#else /* LOG_ENABLED not defined */

/// disabled: dead code branch preserves format checking and variable
/// references (prevents -Wunused warnings) while generating zero
/// instructions in the compiled binary.
#define LOG(...) \
  do { if (0) { fprintf(stderr, __VA_ARGS__); } } while (0)
#define LOG_ERROR(...) \
  do { if (0) { fprintf(stderr, __VA_ARGS__); } } while (0)
#define LOG_PHASE(...) \
  do { if (0) { fprintf(stderr, __VA_ARGS__); } } while (0)
#define LOG_STEP(step, x, y, dir) \
  do { (void)(step); (void)(x); (void)(y); (void)(dir); } while (0)

#endif /* LOG_ENABLED */
#endif /* LOG_H */
