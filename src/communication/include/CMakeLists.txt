# 添加源文件
file(GLOB_RECURSE COMMON_LIST
    "*.h"
    "*.cpp"
)

# 创建库
add_library(common SHARED ${COMMON_LIST})

# 安装库
install(TARGETS common
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)