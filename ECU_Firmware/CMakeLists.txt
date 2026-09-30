# Torque vectoring ECU firmware.
#
#   cmake --preset default   STM32G474 firmware (cross compile, see cmake/arm-none-eabi.cmake)
#   cmake --preset sim       host build: ecu_sim for HIL/SIL runs plus the tests
#
cmake_minimum_required(VERSION 3.20)
project(tv_ecu C)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(APP_SRC
  app/app.c
  app/console.c
  app/derate.c
  app/driver_inputs.c
  app/gps.c
  app/hil.c
  app/imu.c
  app/motors.c
  app/torque_vectoring.c
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set(WARN -Wall -Wextra -Wshadow -Wdouble-promotion)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Generic")
  # ---- STM32G474 firmware ----
  enable_language(ASM)
  set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
  set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
  file(GLOB TARGET_SRC CONFIGURE_DEPENDS hal/stm32g474/*.c)

  add_executable(${PROJECT_NAME}.elf ${APP_SRC} ${TARGET_SRC} hal/stm32g474/startup_stm32g474retx.s)
  target_include_directories(${PROJECT_NAME}.elf PRIVATE app hal hal/stm32g474 hal/stm32g474/cmsis)
  target_compile_definitions(${PROJECT_NAME}.elf PRIVATE STM32G474xx)
  target_compile_options(${PROJECT_NAME}.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${WARN} -fsingle-precision-constant -fno-math-errno>)
  target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/hal/stm32g474/STM32G474CE.ld
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,-u,_printf_float
  )
  target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)

  add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
  )

  set(PROGRAMMER STM32_Programmer_CLI CACHE STRING "STM32CubeProgrammer CLI")
  add_custom_target(flash
    COMMAND ${PROGRAMMER} -c port=SWD mode=UR -w ${PROJECT_NAME}.elf -v -rst
    DEPENDS ${PROJECT_NAME}.elf
    USES_TERMINAL
  )
else()
  # ---- host: ecu_sim and tests ----
  add_library(ecu_sil STATIC ${APP_SRC} sim/sim.c)
  target_include_directories(ecu_sil PUBLIC app hal sim)
  target_compile_options(ecu_sil PUBLIC ${WARN})
  if(MINGW)
    target_link_options(ecu_sil PUBLIC -static)
  endif()
  if(MSVC)
    target_compile_definitions(ecu_sil PUBLIC _CRT_SECURE_NO_WARNINGS)
  else()
    target_link_libraries(ecu_sil PUBLIC m)
  endif()

  add_executable(ecu_sim sim/ecu_sim.c sim/tcp.c)
  target_link_libraries(ecu_sim PRIVATE ecu_sil)
  if(WIN32)
    target_link_libraries(ecu_sim PRIVATE ws2_32)
  endif()

  enable_testing()
  file(GLOB TESTS CONFIGURE_DEPENDS tests/test_*.c)
  foreach(src ${TESTS})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_include_directories(${name} PRIVATE tests)
    target_link_libraries(${name} PRIVATE ecu_sil)
    add_test(NAME ${name} COMMAND ${name})
  endforeach()

  # The real motor controller firmware against this ECU, when its repo is next to this one:
  #   cmake --preset sim -DMC_FIRMWARE_DIR=../../STM32-Motor-Controller/controller_firmware
  set(MC_FIRMWARE_DIR "" CACHE PATH "STM32-Motor-Controller/controller_firmware, for the interop test")
  if(MC_FIRMWARE_DIR)
    get_filename_component(MC ${MC_FIRMWARE_DIR} ABSOLUTE BASE_DIR ${CMAKE_SOURCE_DIR})
    file(GLOB MC_APP_SRC ${MC}/app/*.c)
    add_library(mc_sil STATIC ${MC_APP_SRC} ${MC}/sim/sim.c ${MC}/sim/motor_model.c)
    target_include_directories(mc_sil PUBLIC ${MC}/app ${MC}/hal ${MC}/sim)
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
      target_compile_options(mc_sil PRIVATE -fsingle-precision-constant)
    endif()
    if(NOT MSVC)
      target_link_libraries(mc_sil PUBLIC m)
    endif()

    add_executable(motor_interop tests/interop/motor_interop.c sim/tcp.c)
    target_link_libraries(motor_interop PRIVATE mc_sil)
    if(WIN32)
      target_link_libraries(motor_interop PRIVATE ws2_32)
    endif()
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME motor_interop COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_SOURCE_DIR}/tests/interop/run_interop.py $<TARGET_FILE:ecu_sim> $<TARGET_FILE:motor_interop>)
  endif()
endif()
