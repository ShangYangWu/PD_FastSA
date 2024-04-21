CC=g++
LDFLAGS=-std=c++11 -O3 -lm
SOURCES=src/floorplanner.cpp src/main.cpp src/module.cpp src/parser.cpp src/contour.cpp src/print.cpp src/perturb.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fp
INCLUDES=src/module.h src/floorplanner.h

all: $(SOURCES) bin/$(EXECUTABLE)

bin/$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o:  %.c  ${INCLUDES}
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o bin/$(EXECUTABLE)
