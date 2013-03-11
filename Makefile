TARGET=ece486
OBJS=init486.o err486.o sample486.o blinkandwait.o config_mp45dt02.o

# gcc development tools install directory for STM32F4xx processors
armbase = /usr/local/arm-hard/
#armbase = /usr/linux/arm-hard/


CC=$(armbase)/bin/arm-eabi-gcc
LD=$(armbase)/bin/arm-eabi-ld
AS=$(armbase)/bin/arm-eabi-as
AR=$(armbase)/bin/arm-eabi-ar
RANLIB=$(armbase)/bin/arm-eabi-ranlib
OBJCOPY=$(armbase)/bin/arm-eabi-objcopy

CFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -O3 -W -Wall -nostdlib -fomit-frame-pointer -fno-strict-aliasing -fdata-sections -I. -I../stm32f4 -I../libperiph -I../cm3 -I../libdsp -include stm32f4xx_conf.h -mfpu=fpv4-sp-d16 -DARM_MATH_CM4
LDFLAGS=-nostdlib -Wno-override-init -Wl,-Map,arm.map -Wl,-Tlinkage.lds -Wl,--gc-sections

lib${TARGET}.a: ${OBJS}
	${AR} rc lib${TARGET}.a ${OBJS}
	${RANLIB} lib${TARGET}.a

clean:
	rm -f *.o ${TARGET}
