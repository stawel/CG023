
set(exec logger)

set(${exec}_SRCS  main.cpp)
#set(${exec}_HDRS  logger.h)
set(${exec}_PORT  /dev/ttyUSB0)
set(${exec}_SERIAL picocom @SERIAL_PORT@ -b 115200 -l)

set(${exec}_BOARD nano)
set(ARDUINO_CPUMENU .menu.cpu.atmega328)

generate_arduino_firmware(${exec})

