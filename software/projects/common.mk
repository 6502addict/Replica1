# Requires cc65 toolchain installed
CC = cc65
AS = ca65
AR = ar65
CC65_HOME = /usr/local/share/cc65

# Project root paths - common.mk location determines root
PROJECT_ROOT = $(abspath ../..)
TARGET       = $(or $(MACHINE6502), max6502)
INCLUDES     = -I $(PROJECT_ROOT)/include
LIB_DIR      = $(PROJECT_ROOT)/lib
INCLUDE_DIR  = $(PROJECT_ROOT)/include
MAN_DIR      = $(PROJECT_ROOT)/man
DOC_DIR      = $(PROJECT_ROOT)/man
TESTS        = $(PROJECT_ROOT)/tests
