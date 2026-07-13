# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/oz/.platformio/packages/framework-espidf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "/Users/oz/.platformio/packages/framework-espidf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader"
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix"
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/tmp"
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/src/bootloader-stamp"
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/src"
  "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/oz/src/hoverboard-firmware-hack-FOC-UART-to-MavLink-esp32/.pio/build/esp32dev/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
