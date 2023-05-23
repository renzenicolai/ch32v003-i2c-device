all : flash

TARGET:=main

CFLAGS+=-DTINYVECTOR
#ADDITIONAL_C_FILES+=

PREFIX := riscv64-elf
CH32V003FUN := ch32v003fun
MINICHLINK := minichlink

include ch32v003fun/ch32v003fun.mk

flash : cv_flash
clean : cv_clean

