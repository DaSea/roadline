# 版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (roadtools)

# 查找当前目录下的所有文件，并将名称保存到DIR_SRCS
aux_source_directory (. DIR_SRCS)

# 头文件查找
include_directories (./)

# 添加src子目录, 子目录必须有CMakeList.txt
# add_subdirectory(src)

# 指定生成目标
add_executable (roadtools main.cpp ${DIR_SRCS})

# 添加链接库

