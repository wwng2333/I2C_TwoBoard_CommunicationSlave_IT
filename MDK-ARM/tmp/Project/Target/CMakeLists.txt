# CMSIS Build CMakeLists generated on 2023-07-16T11:11:50

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET Project)
set(CPU Cortex-M0+)
set(PRJ_DIR "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM")
set(OUT_DIR "C:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/out/Project/Target")
set(INT_DIR "C:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/tmp/Project/Target")
set(FPU NO_FPU)
set(SECURE Non-secure)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE none)
set(DEBUG on)
set(CC_FLAGS_GLOBAL "-fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -std=c99 -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -std=c99 -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--library_type=microlib --map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/Project_Target.sct")

set(DEFINES
  _RTE_
  PY32F002Ax5
  USE_FULL_LL_DRIVER
  __MICROLIB
)

set(INC_PATHS
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Inc"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/BSP/PY32F002xx_Start_Kit"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/CMSIS/Include"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/CMSIS/Device/PY32F0xx/Include"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/PY32F0xx_HAL_Driver/Inc"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/RTE/_Target"
  "C:/Users/ander/AppData/Local/Arm/Packs/ARM/CMSIS/5.9.0/CMSIS/Core/Include"
  "C:/Users/ander/AppData/Local/arm/packs/Puya/PY32F0xx_DFP/1.1.0/Device/Include"
)

set(AS_ARM_SRC_FILES
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/startup_py32f002xx.s"
)

set(CC_SRC_FILES
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/BSP/PY32F002xx_Start_Kit/py32f002xx_ll_Start_Kit.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/PY32F0xx_HAL_Driver/Src/py32f0xx_ll_gpio.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/PY32F0xx_HAL_Driver/Src/py32f0xx_ll_i2c.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/PY32F0xx_HAL_Driver/Src/py32f0xx_ll_rcc.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Drivers/PY32F0xx_HAL_Driver/Src/py32f0xx_ll_utils.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Inc/SEGGER_RTT.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Inc/SEGGER_RTT_printf.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Src/main.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Src/py32f0xx_it.c"
  "c:/Users/ander/Downloads/PYMCU_20221117/Library and Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/Src/system_py32f0xx.c"
)

set(AS_FLAGS_c:/Users/ander/Downloads/PYMCU_20221117/Library?and?Examples/PY32F0xx_Firmware_V1.1.2/Projects/PY32F002A-STK/Example_LL/I2C/I2C_TwoBoard_CommunicationSlave_IT_Init/MDK-ARM/startup_py32f002xx.s "-masm=auto")

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "c:/cadence166/Cadence/SPB_Data/.vcpkg/artifacts/fd3d78d7/compilers.arm.armclang/6.20.0/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.20.0")
set(TOOLCHAIN_VERSION_MIN "6.20.0")
include ("C:/cadence166/Cadence/SPB_Data/.vcpkg/artifacts/fd3d78d7/tools.open.cmsis.pack.cmsis.toolbox/2.0.0/etc/AC6.6.18.0.cmake")
include ("C:/cadence166/Cadence/SPB_Data/.vcpkg/artifacts/fd3d78d7/tools.open.cmsis.pack.cmsis.toolbox/2.0.0/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_SECURE} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_SECURE} ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  string(REPLACE " " "?" S ${SRC})
  if(DEFINED AS_FLAGS_${S})
    set(AS_FLAGS_LOCAL "${AS_FLAGS_${S}}")
  else()
    set(AS_FLAGS_LOCAL "${AS_FLAGS_GLOBAL}")
  endif()
  set_source_files_properties(${SRC} PROPERTIES COMPILE_FLAGS "${AS_FLAGS_LOCAL}")
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "Project")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
