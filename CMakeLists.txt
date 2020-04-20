cmake_minimum_required(VERSION 3.3)

project(can2db LANGUAGES C)

include (GNUInstallDirs)
INCLUDE(FindPkgConfig)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
  add_definitions(-D_GNU_SOURCE)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-parentheses")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-strict-aliasing")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSO_RXQ_OVFL=40")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPF_CAN=29")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAF_CAN=PF_CAN")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DN_SLCAN=17")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSCM_TIMESTAMPING_OPT_STATS=54")

include_directories (.)
include_directories (./include)

set(PROGRAMS_REDIS 
  can2redis
)
set(PROGRAMS_HTTP
  can2atlas
)
set(PROGRAMS_CANLIB
    ${PROGRAMS_REDIS}
    ${PROGRAMS_HTTP}
)

set(PROGRAMS
    ${PROGRAMS_CANLIB}
)

add_library(can STATIC
    include/can-utils/lib.c
    include/can-utils/canframelen.c
)

foreach(name ${PROGRAMS})
  add_executable(${name} ${name}.c)

  if("${name}" IN_LIST PROGRAMS_J1939)
    target_link_libraries(${name}
        PRIVATE j1939
    )
  elseif("${name}" IN_LIST PROGRAMS_CANLIB)
    target_link_libraries(${name}
        PRIVATE can
    )
  endif()

  if("${name}" STREQUAL ${PROGRAMS_REDIS} )
    target_link_libraries(${name} PRIVATE hiredis)
  endif()

  if("${name}" STREQUAL ${PROGRAMS_HTTP} )
    target_link_libraries(${name} PRIVATE json-c)
    target_link_libraries(${name} PRIVATE m)
  endif()

  install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()


ADD_CUSTOM_TARGET(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_SOURCE_DIR}/cmake/make_uninstall.cmake")
