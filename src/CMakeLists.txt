cmake_minimum_required(VERSION 3.14)
project(plt)
 
set(CMAKE_CXX_STANDARD 11)
 
add_executable(plt main.cpp)
add_definitions("-Wno-dev")

# Find Packages
find_package( PCL 1.7 REQUIRED COMPONENTS common io)

if( PCL_FOUND  )
  include_directories( ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )  
  add_definitions( -DPCL_NO_PRECOMPILE )

  link_directories( ${PCL_LIBRARY_DIRS} )
  target_link_libraries(plt ${PCL_LIBRARIES})
#   target_link_libraries( test ${PCL_LIBRARIES} )
endif()
 
# For OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# add_definitions("-DWITHOUT_NUMPY")
# link_directories(python2.7) #-I/usr/local/include/python2.7 -lpython2.7
# include_directories(/usr/local/include/python2.7 )
# matplotlibcppで使うPythonとNumpyを探す
# find_package(Python3 COMPONENTS Development NumPy REQUIRED)
# find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(PythonLibs 3.0 REQUIRED)
# include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})
# target_include_directories(plt PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
# target_link_libraries(plt Python3::Python Python3::NumPy)