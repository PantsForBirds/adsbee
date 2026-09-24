target_sources(${PROJECT_NAME} PRIVATE
    "lr2021.cpp"
    "lr2021_async.cpp"
    "lr2021_irq_drain.cpp"
    "lr2021_ll.cpp"
    "lr2021_ook.cpp"
    "lr2021_regmem.cpp"
    "lr2021_common.cpp"
    "lr2021_system.cpp"
    "lr2021_fifo.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
)
