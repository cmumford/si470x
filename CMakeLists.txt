# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.9)

project(si470x VERSION 0.8 LANGUAGES C CXX)

set(RDS_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rds")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED OFF)
  set(CMAKE_C_EXTENSIONS OFF)
endif(NOT CMAKE_C_STANDARD)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif(NOT CMAKE_CXX_STANDARD)

include(CheckIncludeFile)
check_include_file("wiringPi.h" HAVE_WIRING_PI_H)
check_include_file("linux/i2c-dev.h" HAVE_I2C_DEV_H)

include(CheckLibraryExists)
check_library_exists(wiringPi wiringPiSetupGpio "" HAVE_WIRINGPI)

include(CheckCXXSymbolExists)
check_cxx_symbol_exists(usleep "unistd.h" HAVE_USLEEP)
check_cxx_symbol_exists(nanosleep "time.h" HAVE_NANOSLEEP)
check_cxx_symbol_exists(perror "stdio.h" HAVE_PERROR)

if (HAVE_WIRING_PI_H)
  add_definitions(-DHAVE_WIRING_PI)
endif (HAVE_WIRING_PI_H)
if (HAVE_I2C_DEV_H)
  add_definitions(-DHAVE_I2C_DEV)
endif (HAVE_I2C_DEV_H)
if (HAVE_USLEEP)
  add_definitions(-DHAVE_USLEEP)
endif (HAVE_USLEEP)
if (HAVE_NANOSLEEP)
  add_definitions(-DHAVE_NANOSLEEP)
endif (HAVE_NANOSLEEP)
if (HAVE_PERROR)
  add_definitions(-DHAVE_PERROR)
endif (HAVE_PERROR)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

link_directories(${RDS_LIB_DIR}/build)
add_library(si470x "")
target_sources(si470x
  PRIVATE
    "include/si470x.h"
    "src/port_unix.c"
    "src/si470x.c"
    "src/si470x_misc.c"
    "src/si470x_misc.h"
)
target_include_directories(si470x
  PUBLIC
    $<BUILD_INTERFACE:${RDS_LIB_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_options(si470x PRIVATE -Werror -Wall -Wextra)
