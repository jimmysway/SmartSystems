# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jason/esp/esp-idf/components/bootloader/subproject"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/tmp"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/src"
  "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jason/Documents/Schoolwork/Fall_2023/EC444/Team7-Lee-Li-Slobodchikov-Sui/quest-3/code/Wireless-Carmin-Watch/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
