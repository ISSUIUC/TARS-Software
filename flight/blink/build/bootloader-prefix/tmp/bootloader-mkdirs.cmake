# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/aidancostello/esp/esp-idf/components/bootloader/subproject"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/tmp"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/src"
  "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/aidancostello/Github/TARS-Software/flight/blink/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
