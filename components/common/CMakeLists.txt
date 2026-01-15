idf_component_register(
    # 如果 common 文件夹里目前只有头文件，SRCS 可以留空
    SRCS "" 
    # 只要把当前目录加入 INCLUDE_DIRS，其他人就能搜到它
    INCLUDE_DIRS "." 
    REQUIRES
)