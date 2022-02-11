# ==========================================
#   Unity Project - A Test Framework for C
#   Copyright (c) 2007 Mike Karlesky, Mark VanderVoord, Greg Williams
#   [Released under MIT License. Please refer to license.txt for details]
# ==========================================

#We try to detect the OS we are running on, and adjust commands as needed
ifeq ($(OS),Windows_NT)
  ifeq ($(shell uname -s),) # not in a bash-like shell
	CLEANUP = del /F /Q
	MKDIR = mkdir
  else # in a bash-like shell, like msys
	CLEANUP = rm -f
	MKDIR = mkdir -p
  endif
	TARGET_EXTENSION=.exe
else
	CLEANUP = rm -f
	MKDIR = mkdir -p
	TARGET_EXTENSION=.out
endif

C_COMPILER=g++
ifeq ($(shell uname -s), Darwin)
C_COMPILER=clang
endif

UNITY_ROOT=../Unity

# CFLAGS=-std=c89
CFLAGS = -Wall
CFLAGS += -Wextra
CFLAGS += -Wpointer-arith
CFLAGS += -Wcast-align
CFLAGS += -Wwrite-strings
CFLAGS += -Wswitch-default
CFLAGS += -Wunreachable-code
CFLAGS += -Winit-self
CFLAGS += -Wmissing-field-initializers
CFLAGS += -Wno-unknown-pragmas
# CFLAGS += -Wstrict-prototypes
CFLAGS += -Wundef
CFLAGS +=-DUNIT_TEST
# CFLAGS += -Wold-style-definition
#CFLAGS += -Wno-misleading-indentation

TARGET_BASE1=test1
TARGET_BASE2=test2
TARGET_BASE3=test3
TARGET1 = $(TARGET_BASE1)$(TARGET_EXTENSION)
TARGET2 = $(TARGET_BASE2)$(TARGET_EXTENSION)
TARGET3 = $(TARGET_BASE3)$(TARGET_EXTENSION)
SRC_FILES1=$(UNITY_ROOT)/src/unity.c ../src/throttle.cpp ../src/stm32_vcu.cpp ../src/utils.cpp test_throttle.cpp  ../libopeninv/src/params.cpp ../libopeninv/src/my_string.c ../libopeninv/src/errormessage.cpp test_runners/test_throttle_Runner.c
SRC_FILES2=$(UNITY_ROOT)/src/unity.c ../src/GS450H.cpp test_GS450H.cpp test_runners/test_GS450H_Runner.c
SRC_FILES3=$(UNITY_ROOT)/src/unity.c ../src/temp_meas.cpp ../src/stm32_vcu.cpp test_temp_meas.cpp test_runners/test_temp_meas_Runner.c ../libopeninv/src/params.cpp ../libopeninv/src/my_string.c ../libopeninv/src/errormessage.cpp ../src/throttle.cpp
INC_DIRS=-Isrc -I$(UNITY_ROOT)/src -I../include -I../libopeninv/include
SYMBOLS=

all: clean default

default: $(SRC_FILES1) $(SRC_FILES2)
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES1) -o $(TARGET1)
	- ./$(TARGET1)
	$(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES3) -o $(TARGET3)
	- ./$(TARGET3)

test_runners/test_throttle_Runner.c: test_throttle.cpp
	ruby $(UNITY_ROOT)/auto/generate_test_runner.rb test_throttle.cpp  test_runners/test_throttle_Runner.c
test_runners/test_GS450H_Runner.c: test_GS450H.cpp
	ruby $(UNITY_ROOT)/auto/generate_test_runner.rb test_GS450H.cpp test_runners/test_GS450H_Runner.c
test_runners/test_temp_meas_Runner.c: test_temp_meas.cpp
	ruby $(UNITY_ROOT)/auto/generate_test_runner.rb test_temp_meas.cpp test_runners/test_temp_meas_Runner.c

clean:
	$(CLEANUP) $(TARGET1)
	$(CLEANUP) $(TARGET3)

ci: CFLAGS += -Werror
ci: default

	# $(C_COMPILER) $(CFLAGS) $(INC_DIRS) $(SYMBOLS) $(SRC_FILES2) -o $(TARGET2)
	# - ./$(TARGET2)
