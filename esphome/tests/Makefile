CXX ?= g++
COMMON = -std=c++17 -O2 -Wall -Wextra -Werror -Wshadow -I../components
# The ESP32-S3 FPU is single precision: flag any accidental double maths in firmware code.
FIRMWARE_FLAGS = $(COMMON) -Wdouble-promotion -Wconversion -Wno-sign-conversion
COMPONENT = ../components/smartknob

test: test_smartknob
	./test_smartknob

detent_engine.o: $(COMPONENT)/detent_engine.cpp $(COMPONENT)/detent_engine.h
	$(CXX) $(FIRMWARE_FLAGS) -c -o $@ $<

foc_math_check.o: foc_math_check.cpp $(COMPONENT)/foc_math.h
	$(CXX) $(FIRMWARE_FLAGS) -c -o $@ $<

test_smartknob: test_smartknob.cpp detent_engine.o foc_math_check.o $(COMPONENT)/foc_math.h
	$(CXX) $(COMMON) -o $@ test_smartknob.cpp detent_engine.o foc_math_check.o

clean:
	rm -f test_smartknob *.o

.PHONY: test clean
