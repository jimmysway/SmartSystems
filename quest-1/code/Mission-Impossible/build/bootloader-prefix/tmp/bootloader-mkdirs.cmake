# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jakelee/esp/esp-idf/components/bootloader/subproject"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/tmp"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/src"
  "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jakelee/Documents/Boston-Univeristy/EC444/Lee-Jake/skills/cluster-1/12/code/Polling-Loop-Program/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
