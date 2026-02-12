# Compiler can be overridden: make CC="zig cc"
CC ?= gcc
CFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic

# Logging enabled by default, disable with: make LOG_ENABLED=false
LOG_ENABLED ?= true
ifeq ($(LOG_ENABLED),true)
  CFLAGS += -DLOG_ENABLED
endif

SUFFIX = .out
VERSIONS = ffv0 ffv1 ffv2 ffv3
TARGETS = $(addsuffix $(SUFFIX), $(VERSIONS))

all: $(TARGETS)

%$(SUFFIX): src/%.c src/api.h src/log.h
	$(CC) $(CFLAGS) -O2 -o $@ $<

# Pattern rule to allow 'make ffv1' to build 'ffv1.out'
$(VERSIONS): %: %$(SUFFIX)

run:
	@if [ -z "$(filter-out run,$(MAKECMDGOALS))" ]; then \
		echo "Usage: make run <target>"; \
		echo "Example: make run ffv1"; \
		exit 1; \
	fi
	@target="$(filter-out run,$(MAKECMDGOALS))"; \
	if [ -f "$$target$(SUFFIX)" ]; then \
		./$$target$(SUFFIX); \
	else \
		echo "Error: $$target$(SUFFIX) not found. Run 'make $$target' first."; \
		exit 1; \
	fi

debug:
	$(CC) $(CFLAGS) -O0 -g -DDEBUG -o ffv0.out src/ffv0.c
	$(CC) $(CFLAGS) -O0 -g -DDEBUG -o ffv1.out src/ffv1.c
	$(CC) $(CFLAGS) -O0 -g -DDEBUG -o ffv2.out src/ffv2.c
	$(CC) $(CFLAGS) -O0 -g -DDEBUG -o ffv3.out src/ffv3.c

clean:
	rm -f *.out *.o

.PHONY: all debug clean run

# Catch-all for run arguments
%:
	@:
