CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=c11 -Isrc
LDFLAGS = -lraylib -lm

# Platform-specific raylib link flags
UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
    LDFLAGS += -lGL -lpthread -ldl -lrt -lX11
endif
ifeq ($(UNAME), Darwin)
    LDFLAGS += -framework OpenGL -framework Cocoa -framework IOKit
endif

# If raylib is not installed system-wide, point to a local build:
# make RAYLIB_DIR=/path/to/raylib-5.0_linux_amd64
ifdef RAYLIB_DIR
    CFLAGS  += -I$(RAYLIB_DIR)/include
    LDFLAGS := -L$(RAYLIB_DIR)/lib $(LDFLAGS)
endif

TARGET  = orbital-sim
SRCS    = src/main.c src/physics.c src/solver.c src/trail.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean