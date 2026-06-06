set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Remove these two lines - CMake detects them automatically:
# set(CMAKE_C_COMPILER_ID GNU)    ← DELETE
# set(CMAKE_CXX_COMPILER_ID GNU)  ← DELETE

set(TOOLCHAIN_PATH "C:/Users/Goran/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin")
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Use absolute paths + CACHE FILEPATH FORCE so they appear in CMakeCache.txt
set(CMAKE_C_COMPILER    "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc.exe" CACHE FILEPATH "C compiler"   FORCE)
set(CMAKE_CXX_COMPILER  "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}g++.exe" CACHE FILEPATH "C++ compiler" FORCE)
set(CMAKE_ASM_COMPILER  "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc.exe" CACHE FILEPATH "ASM compiler" FORCE)
set(CMAKE_LINKER        "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}g++.exe" CACHE FILEPATH "Linker"       FORCE)
set(CMAKE_OBJCOPY       "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}objcopy.exe")
set(CMAKE_SIZE          "${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}size.exe")

# Rest of your file stays the same...
set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32L431XX_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")