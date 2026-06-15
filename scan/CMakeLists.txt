cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(PointCloudToMesh)

# PCLライブラリを探して読み込む
find_package(PCL 1.2 REQUIRED)

# 必要な設定を適用する
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# main.cpp を元に「PointCloudToMesh」という実行ファイルを作る
add_executable(PointCloudToMesh main01_Scan.cpp)
target_link_libraries(PointCloudToMesh ${PCL_LIBRARIES})
