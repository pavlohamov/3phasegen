
CFLAGS += \
	-I./dbg \
	-I./inc \
	-I./src \
	-I./lib/ \
	
CFLAGS += \
	-DDEBUG \
	-DSTM32F070x6 \
	-DHSE_VALUE=8000000 \
	-DUSE_HAL_DRIVER \
	-DUSE_FULL_ASSERT \
	\
	-I./sdk/include/ \
	-I./sdk/include/arm \
	-I./sdk/include/cmsis/ \
	-I./sdk/include/cortexm/ \
	-I./sdk/include/diag \
	-I./sdk/include/stm32f0-stdperiph \
	
export SRC := \
	$(wildcard ./src/*.c*) \
	$(wildcard ./dbg/*.c*) \
	$(wildcard ./lib/src/*.c) \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_cortex.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_usart.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_ll_rcc.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_rcc.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_rcc_ex.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_ll_gpio.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_gpio.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_ll_adc.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_adc.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_adc_ex.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_iwdg.c \
	\
	./sdk/src/stm32f0-stdperiph/stm32f0xx_ll_tim.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_tim.c \
	./sdk/src/stm32f0-stdperiph/stm32f0xx_hal_tim_ex.c \

