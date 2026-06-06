cmake_minimum_required(VERSION 3.20)

# 跳过编译器测试编译（嵌入式交叉编译器无法独立链接可执行文件）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(gm6020_control C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ============ 编译选项 ============
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -DSTM32F407xx
    -DUSE_HAL_DRIVER
    -D__FPU_PRESENT=1
    -O2
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wno-unused-parameter
)

# ============ 链接选项 ============
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGHX_FLASH.ld)

add_link_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    --specs=nosys.specs
    -u _printf_float
)

# ============ 头文件路径 ============
set(INCLUDE_DIRS
    Core/Inc
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# ============ 源文件 ============
file(GLOB_RECURSE HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

# FreeRTOS 源文件（手动列出，避免 portable 目录冲突）
set(FREERTOS_CORE_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
)

set(FREERTOS_PORTABLE_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

set(FREERTOS_HEAP_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
)

set(FREERTOS_CMSIS_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
)

set(APP_SOURCES
    Core/Src/main.c
    Core/Src/freertos.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/system_stm32f4xx.c
    Core/Src/pid.c
    Core/Src/gm6020.c
    Core/Src/gm6020_task.c
    Core/Src/gm6020_cmd.c
)

set(ASM_SOURCES
    Core/Src/startup_stm32f407xx.s
    Core/Src/syscalls.c
    Core/Src/sysmem.c
)

# ============ 目标 ============
add_executable(${CMAKE_PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_CORE_SOURCES}
    ${FREERTOS_PORTABLE_SOURCES}
    ${FREERTOS_HEAP_SOURCES}
    ${FREERTOS_CMSIS_SOURCES}
    ${ASM_SOURCES}
)

# Blinky 测试目标（独立点灯程序，不依赖 HAL/FreeRTOS）
add_executable(blinky.elf
    Core/Src/blinky.c
    Core/Src/startup_stm32f407xx.s
    Core/Src/system_stm32f4xx.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
)

target_include_directories(${CMAKE_PROJECT_NAME}.elf PRIVATE ${INCLUDE_DIRS})
target_include_directories(blinky.elf PRIVATE ${INCLUDE_DIRS})

# Blinky 生成 hex
add_custom_command(TARGET blinky.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex blinky.elf blinky.hex
    COMMENT "Generating blinky.hex"
)

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# ============ 自定义命令：生成 .bin 和 .hex ============
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley ${CMAKE_PROJECT_NAME}.elf
    COMMENT "Generating .hex and .bin files"
)
