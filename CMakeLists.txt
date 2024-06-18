cmake_minimum_required(VERSION 3.10)
project(raspberry_motors)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

######################
### help functions ###
######################
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

####################
### dependencies ###
####################

# from the internet (or even don't have in
set(Dependencies)

#Il primo elemento è un percorso a una cartella che contiene un altro cmake che vuoi incorporare. Il secondo argomento indica dove vuoi scaricare i file di quel cmake

add_subdirectory(pybind11)
include_directories(pybind11/include)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hebi-cpp ${CMAKE_CURRENT_SOURCE_DIR}/hebi-cpp/build) 

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../thirdParty/Eigen3) #sta aggiungendo a CMAKE_PREFIX_PATH (che è una env var) un percorso in cui probabilmente vi saranno dei .cmake file, ovvero file che permettono di aggiungere progetti esterni (le loro librerie, e shared library). Questi .cmake file sono costruiti con sudo make install probabilmente. Ora che sa che cosa c'è in Eigen3 e sa che c'è un .cmake relativo a Eigen, posso includerlo nel progetto con find_packages. ALternativamente, questo Eigen3.cmake doveva stare in usr/local/ or usr/share
find_package(Eigen3 CONFIG REQUIRED)

if (APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else ()
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

#find_package(Python 3.6 COMPONENTS Interpreter Development.Module Development.Embed NumPy)


#######################
### src and linking ###
#######################
#lista di directories che stanno in quel percorso, tale lista la chiami SUBDIRS


#Il .so è una libreria precompilata. Devo aggiungere solo quella
#librerie precompilate da includere. Gli so contengono file oggetto e il vantagio di usarli è che le funzioni sono già compilate quindi perdi meno tempo in compilazione. Alternatively, one could use .lib file to connect the code but is a static connection and the functions must be compiled everytime
set(dynamixel_lib ${CMAKE_CURRENT_SOURCE_DIR}/bin/libdxl_x64_cpp.so)  
set(include_dynl ${CMAKE_CURRENT_SOURCE_DIR}/dynamixel_dependencies/include/dynamixel_sdk)       #dove va a cercarsi la directory
#installed by the folder dynamixelSDK

#Alternativo al fare il set del percorso...
#add_library( hebi SHARED IMPORTED )
#set_target_properties( hebi PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../hebi-cpp/build/libhebic++.so)

set(include_hebi  ${CMAKE_CURRENT_SOURCE_DIR}/../hebi-cpp/src)
#set(lib_hebi ${CMAKE_CURRENT_SOURCE_DIR}/../hebi-cpp/build/libhebic++.so)
#set(lib_hebi1 ${CMAKE_CURRENT_SOURCE_DIR}/../hebi-cpp/build/hebic++)
 

pybind11_add_module(prisma_walker Environment.cpp )
target_link_libraries(prisma_walker PUBLIC ${dynamixel_lib} hebi hebic++ )  
target_include_directories(prisma_walker PUBLIC ${include_dynl} ${include_hebi} ${CMAKE_CURRENT_SOURCE_DIR})


