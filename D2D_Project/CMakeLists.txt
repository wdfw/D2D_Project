cmake_minimum_required(VERSION 3.10)
project(D2D_Project)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# 1) 確保 bin 資料夾存在
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# 2) 指定可執行檔輸出到 bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 23) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 確保 result 資料夾存在
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/result)

# 只清空 result 資料夾內容（保留執行檔）
add_custom_target(cleanResult
  COMMENT "清空 result 目錄下的所有檔案，但保留資料夾與執行檔"
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/result
  COMMAND ${CMAKE_COMMAND} -E make_directory  ${CMAKE_SOURCE_DIR}/result
)

add_custom_target(show
  COMMAND ./bin/D2D_Gui ./bin/D2D ./testcase/standard_44/bumps.loc ./testcase/standard_44/rule.txt ./result/
)

add_custom_target(exe
  COMMAND   ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result -p 300 -g 300 -m 1 -c 0.9 -s 51
)

add_custom_target(exe2
  COMMAND ./bin/D2D testcase/unordered_256/bumps.loc testcase/unordered_256/rule.txt ./result -p 100 -g 200 -m 1 -t 4
)

add_custom_target(exe3
  COMMAND ./bin/D2D testcase/holed_100/bumps.loc testcase/holed_100/rule.txt ./result -p 100 -g 200 -m 1 -t 4
)

add_custom_target(exe4
  COMMAND ./bin/D2D testcase/perfect_100//bumps.loc testcase/perfect_100/rule.txt ./result -p 100 -g 200 -m 1 -t 4
)

add_custom_target(exe5
  COMMAND ./bin/D2D testcase/unordered_128/bumps.loc testcase/unordered_128/rule.txt ./result -p 100 -g 200 -m 1 -t 4
)

add_custom_target(exp
  COMMAND  ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result -p 1 -g 1 -m 1 -c 1 -t 4
)


add_custom_target(runall
  COMMAND ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result
)

# for cgal
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(CGAL REQUIRED COMPONENTS Qt5)
# find_package(OpenMP REQUIRED)

# 自動抓取 main.cpp 以及 src 資料夾底下所有 .cpp
file(GLOB_RECURSE MAIN_SOURCES 
    "${CMAKE_SOURCE_DIR}/src/main.cpp" #1
    # "${CMAKE_SOURCE_DIR}/src/ShowGraph.cpp" #2
    # "${CMAKE_SOURCE_DIR}/src/GenerateCase.cpp" #3
    "${CMAKE_SOURCE_DIR}/libs/*.cpp" 
)
#ShowGraph
#ShowResult  

# file(GLOB_RECURSE GEN_SOURCES
#     "${CMAKE_SOURCE_DIR}/src/BumpGenerator.cpp"
#     "${CMAKE_SOURCE_DIR}/include/*.cpp"
# )
# 添加頭文件目錄
include_directories(${CMAKE_SOURCE_DIR}/include)

# 添加可執行檔案
add_executable(D2D ${MAIN_SOURCES}) #1
# add_executable(ShowResult  ${MAIN_SOURCES}) #2
# add_executable(GenerateCase ${MAIN_SOURCES}) #3

# 連結 Qt5 Widgets 和 CGAL
target_link_libraries(D2D CGAL::CGAL Qt5::Widgets ) #1OpenMP::OpenMP_CXX
# target_link_libraries(ShowResult CGAL::CGAL Qt5::Widgets) #2 
# target_link_libraries(GenerateCase CGAL::CGAL Qt5::Widgets) #3

add_subdirectory("${CMAKE_SOURCE_DIR}/D2D_Gui")