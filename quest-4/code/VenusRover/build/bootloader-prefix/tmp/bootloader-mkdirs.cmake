# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jakelee/esp/esp-idf/components/bootloader/subproject"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/tmp"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/src"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jakelee/Documents/Boston-Univeristy/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-4/code/VenusRover/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
