# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/tyler-jmz/esp/v5.5/esp-idf/components/bootloader/subproject"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/tmp"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/src/bootloader-stamp"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/src"
  "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tyler-jmz/esp/codigos-tyler/examples/lora_e220/gps_tracker/gps_tracker_rx/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
