target_sources(${PROJECT_NAME} PRIVATE
    "settings.cpp"
    "settings_strs.cpp"
    "flash_utils.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
)
