
target_sources(${PROJECT_NAME} PRIVATE
    hardware_unit_tests.cpp
    test_lr2021.cpp
    test_df17_cpu.cpp
    
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
)
