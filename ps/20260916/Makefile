############################################################################
## Copyright (C) 2009 ~ 2017 Beijing Star Test High-Tech Co., Ltd.
##
## All rights reserved by www.startest.cn, Inc.
##
## Makefile : TODO
##
## Maintainer  : $Id: Makefile 19762 2022-07-05 09:01:58Z wucheng $
##
## Author      : WuCheng <wucheng@startest.net>
##
## Create Date : 2022-07-05
############################################################################

.PHONY : all install clean

VERSION = release
VERSION ?= debug

PROJECT_NAME = GB2400G-DRY 
EDITION = 00.00.01

MKDIR = mkdir
RM = rm
RMFLAGS = -fr

#CC = /home/lgz/linux_project_GB400M_WET/tools/cross_compiler/CodeSourcery/Sourcery_CodeBench_Lite_for_Xilinx_GNU_Linux/bin/arm-xilinx-linux-gnueabi-gcc
CC = $(CROSS_COMPILE)gcc

ifeq (${VERSION}, debug)
#CCFLAG = -pipe -O0 -ggdb -Wall -W -w -c
CCFLAG = -pipe -Og -g -Wall -W -w -c
else
CCFLAG = -pipe -O2 -Wall -W -w -c
endif

LDBINFLAG = -lrt -lpthread -lm -g

INCDIRPATH = -I. -I./include
LIBDIRPATH = -L./lib

SRC_DIR = src
EXE_DIR = $(SRC_DIR)
OBJ_DIR = .objs
DEP_DIR = .deps
DIRS = $(OBJ_DIR) $(DEP_DIR) 

EXE_BIN = app
EXE_BIN := $(addprefix $(EXE_DIR)/, $(EXE_BIN))

INSTALL_DIR = build/program/app

SRC_LIST = $(wildcard $(SRC_DIR)/*.c)
SRC_LIST := $(subst $(SRC_DIR)/, , $(SRC_LIST))
OBJ_LIST = $(SRC_LIST:.c=.o)
OBJ_LIST := $(addprefix $(OBJ_DIR)/, $(OBJ_LIST))
DEP_LIST = $(SRC_LIST:.c=.dep)
DEP_LIST := $(addprefix $(DEP_DIR)/, $(DEP_LIST))

ifeq ("$(wildcard $(OBJ_DIR))", "")
DEP_DIR_OBJS := $(OBJ_DIR)
endif
ifeq ("$(wildcard $(DEP_DIR))", "")
DEP_DIR_DEPS := $(DEP_DIR)
endif

all: ${EXE_BIN}
	@echo "=========================================================="
	@echo "        Project ${PROJECT_NAME} Build All Done."
	@echo "    [ ${VERSION} edition, version ${EDITION} ]"
	@echo "=========================================================="
	@echo ""

ifneq ($(MAKECMDGOALS), clean)
include $(DEP_LIST)
endif

$(DIRS):
	$(MKDIR) $@

${EXE_BIN} : ${OBJ_LIST}
	@${CC} -o $@ $(filter %.o, $^) ${LIBDIRPATH} ${LDBINFLAG}
	@echo "[M] $@"

${OBJ_DIR}/%.o : ${DEP_DIR_OBJS} $(SRC_DIR)/%.c
	@${CC} ${CCFLAG} ${INCDIRPATH} -o $@ -c $(filter %.c, $^)
	@echo "[M] $@"

${DEP_DIR}/%.dep : ${DEP_DIR_DEPS} $(SRC_DIR)/%.c
	@echo "Creating $@ ..."
	@set -e; \
	${RM} ${RMFLAGS} $@.tmp; \
	${CC} -E -MM ${INCDIRPATH} $(filter %.c, $^) > $@.tmp; \
	sed 's,\(.*\)\.o[ :]*,${OBJ_DIR}/\1.o $@: ,g' < $@.tmp > $@; \
	${RM} ${RMFLAGS} $@.tmp

clean:
	${RM} ${RMFLAGS} ${EXE_BIN} ${DIRS}
