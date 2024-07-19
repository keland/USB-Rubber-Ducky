ifeq ($(OS),Windows_NT)
    CCFLAGS += -D WIN32
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
        CCFLAGS += -D AMD64 -L"C:/Dev-Cpp/lib" -I"C:/Dev-Cpp/include" -I"C:/Dev-Cpp/lib/gcc/mingw32/3.4.2/include"  -I"C:/Dev-Cpp/include/c++/3.4.2/backward"  
-I"C:/Dev-Cpp/include/c++/3.4.2/mingw32"  -I"C:/Dev-Cpp/include/c++/3.4.2"  -I"C:/Dev-Cpp/include" 
    endif
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
        CCFLAGS += -D IA32 -L"C:/Dev-Cpp/lib" -I"C:/Dev-Cpp/include" -I"C:/Dev-Cpp/lib/gcc/mingw32/3.4.2/include"  -I"C:/Dev-Cpp/include/c++/3.4.2/backward"  
-I"C:/Dev-Cpp/include/c++/3.4.2/mingw32"  -I"C:/Dev-Cpp/include/c++/3.4.2"  -I"C:/Dev-Cpp/include"
    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        CCFLAGS += -D LINUX
    endif
    ifeq ($(UNAME_S),Darwin)
        CCFLAGS += -D OSX
    endif
endif

ARCH := $(shell getconf LONG_BIT)

CPP_FLAGS_32 := -D32_BIT -m32
CPP_FLAGS_64 := -D64_BIT

CPP  = g++
CC   = gcc
RES  =
OBJ  = vid_pid_swapper.o $(RES)
LINKOBJ  = vid_pid_swapper.o $(RES) 
BIN = vid_pid_swapper
CXXFLAGS = -O2
CFLAGS = $(CCFLAGS) $(CPP_FLAGS_$(ARCH)) -I /usr/include -L /lib 
RM = rm -f

.PHONY: all all-before all-after clean clean-custom

all: all-before vid_pid_swapper all-after


clean: clean-custom
	${RM} $(OBJ) $(BIN)

vid_pid_swapper: $(OBJ)
	$(CPP) $(LINKOBJ) -o vid_pid_swapper $(LIBS)

vid_pid_swapper.o: vid_pid_swapper.cpp
	$(CPP) -c vid_pid_swapper.cpp -o vid_pid_swapper.o $(CXXFLAGS)

