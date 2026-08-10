# Makefile for libAstroTrac

CC = gcc
CFLAGS = -fPIC -Wall -Wextra -O2 -g -DSB_LINUX_BUILD -I. -I./../../
CPPFLAGS = -fPIC -Wall -Wextra -O2 -g -DSB_LINUX_BUILD -std=gnu++11 -I. -I./../../
LDFLAGS = -shared -lstdc++
RM = rm -f
STRIP = strip
TARGET_LIB = libAstroTrac.so

SRCS = main.cpp AstroTrac.cpp x2mount.cpp
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)

.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} -o $@ $^
	$(STRIP) $@ >/dev/null 2>&1  || true

%.o: %.cpp
	$(CXX) $(CPPFLAGS) -MMD -MP -c -o $@ $<

# Auto-generated per source file above (e.g. AstroTrac.d) - lists exactly which headers each
# .cpp includes, so changing a .h correctly triggers a rebuild of everything that includes it.
-include $(DEPS)

.PHONY: clean
clean:
	${RM} ${TARGET_LIB} ${OBJS} ${DEPS}
