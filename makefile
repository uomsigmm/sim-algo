# Compiler can be overridden: make CC="zig cc"
CC ?= gcc
CFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic
SUFFIX = .out
VERSIONS = ffv1 ffv2 ffv3
TARGETS = $(addsuffix $(SUFFIX), $(VERSIONS))

all: $(TARGETS)

%$(SUFFIX): src/%.c src/api.h
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

debug: CFLAGS += -O0 -g -DDEBUG
debug: $(TARGETS)

clean:
	rm -f *.out *.o

.PHONY: all debug clean run

# Catch-all for run arguments
%:
	@:
