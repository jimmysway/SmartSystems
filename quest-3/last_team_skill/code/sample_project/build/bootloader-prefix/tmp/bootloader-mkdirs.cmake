# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/reman/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/tmp"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/src"
  "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/reman/GitFiles/Team7-Lee-Li-Slobodchikov-Sui/quest-3/last_team_skill/code/sample_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()