# name for output binary files
PROJECT ?= TCD1304-usb-stm32f103

# compiler, objcopy (should be in PATH)
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# path to STM32F103 standard peripheral library
STD_PERIPH_LIBS ?= ../
PROJECT_SRC = src
STM_SRC = ../Libraries/STM32F10x_StdPeriph_Driver/src
STM_SRC += ../Libraries/STM32_USB-FS-Device_Driver/src
STM_SRC += ../Utilities/STM32_EVAL/STM3210B_EVAL

vpath %.c $(PROJECT_SRC)
vpath %.c $(STM_SRC)

# list of source files

SOURCES  = main.c
SOURCES += timer_conf.c
SOURCES += ADC_conf.c
#SOURCES += system_stm32f10x.c
SOURCES += usb_desc.c
SOURCES += usb_endp.c
SOURCES += usb_istr.c
SOURCES += usb_prop.c
SOURCES += usb_pwr.c
SOURCES += hw_config.c
SOURCES += stm32_it.c
#SOURCES += virtualComPort.c



EXT_SRCS = misc.c
EXT_SRCS += stm32f10x_adc.c
#EXT_SRCS += stm32f10x_bkp.c
#EXT_SRCS += stm32f10x_can.c
#EXT_SRCS += stm32f10x_cec.c
#EXT_SRCS += stm32f10x_crc.c
#EXT_SRCS += stm32f10x_dac.c
#EXT_SRCS += stm32f10x_dbgr.c
EXT_SRCS += stm32f10x_dma.c
EXT_SRCS += stm32f10x_exti.c
#EXT_SRCS += stm32f10x_flash.c
#EXT_SRCS += stm32f10x_fsmc.c
EXT_SRCS += stm32f10x_gpio.c
EXT_SRCS += stm32f10x_i2c.c
#EXT_SRCS += stm32f10x_iwdg.c
#EXT_SRCS += stm32f10x_pwr.c
EXT_SRCS += stm32f10x_rcc.c
#EXT_SRCS += stm32f10x_rtc.c
#EXT_SRCS += stm32f10x_sdio.c
EXT_SRCS += stm32f10x_spi.c
EXT_SRCS += stm32f10x_tim.c
#EXT_SRCS += stm32f10x_usart.c
#EXT_SRCS += stm32f10x_wwdg.c



EXT_SRCS += usb_core.c
EXT_SRCS += usb_init.c
EXT_SRCS += usb_int.c
EXT_SRCS += usb_mem.c
EXT_SRCS += usb_regs.c
EXT_SRCS += usb_sil.c

#EXT_SRCS += stm3210b_eval_spi_flash.c
EXT_SRCS += stm3210b_eval.c


SOURCES += $(STD_PERIPH_LIBS)/Libraries/CMSIS/Device/ST/STM32F10x/Source/Templates/system_stm32f10x.c
SOURCES += $(STD_PERIPH_LIBS)/Libraries/CMSIS/Device/ST/STM32F10x/Source/Templates/gcc/startup_stm32f10x_md.s



# specify compiler flags
CFLAGS  = -g -O2 -Wall
CFLAGS += -ggdb -O0
CFLAGS += -T$(STD_PERIPH_LIBS)/Projects/VirtualComport_Loopback/SW4STM32/STM3210B-EVAL/STM32F103VBTx_FLASH.ld
CFLAGS += -march=armv7-m  -mthumb -mcpu=cortex-m3
CFLAGS += -DSTM32F103C8 -DSTM32F10X_MD -DUSE_STM3210B_EVAL -DUSE_STDPERIPH_DRIVER -DUSEUSB #-DSYSCLK_FREQ_56MHz  
CFLAGS += -Wl,--gc-sections -specs=nosys.specs --specs=rdimon.specs
CFLAGS += -I.
CFLAGS += -Isrc
CFLAGS += -Iinc
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/Device/ST/STM32F10x/Include/
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/CMSIS/Include/
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/STM32F10x_StdPeriph_Driver/inc/
CFLAGS += -I$(STD_PERIPH_LIBS)/Libraries/STM32_USB-FS-Device_Driver/inc/
CFLAGS += -I$(STD_PERIPH_LIBS)/Utilities/STM32_EVAL/STM3210B_EVAL/
CFLAGS += -I$(STD_PERIPH_LIBS)/Utilities/STM32_EVAL/Common/

OBJS = $(SOURCES:.c=.o)

EXT_OBJ = $(EXT_SRCS:.c=.o)

all: $(PROJECT).elf

# compile
$(PROJECT).elf: $(SOURCES) $(EXT_OBJ)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin

# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin
