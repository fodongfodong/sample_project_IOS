# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/tmp"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/src/bootloader-stamp"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/src"
  "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/samin_esp_c/sample_project_IOS/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
