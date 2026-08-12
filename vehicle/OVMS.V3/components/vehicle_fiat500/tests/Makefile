CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -g \
            -DFIAT500E_NATIVE_TEST \
            -Imock \
            -I. \
            -I../src \
            -include mock/mock_ovms.hpp

SRC := mock/mock_ovms.cpp \
       ../src/vehicle_fiat500e.cpp \
       test_can_decode.cpp \
       test_transitions.cpp \
       test_commands.cpp \
       test_lifecycle.cpp

all: test

test: $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@
	./$@

clean:
	rm -f test

.PHONY: all test clean
