find_package(GTest REQUIRED)

# 注册一个测试可执行文件的辅助函数
function(add_simrobot_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE simrobot_core GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_subdirectory(core)
