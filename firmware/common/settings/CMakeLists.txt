
if (TARGET application)
    target_sources(application PRIVATE
        settings_strs.cpp
        settings.cpp
        settings_migration.cpp
    )
    target_include_directories(application PRIVATE
        .
    )
elseif (TARGET sub_ghz_radio)
    target_sources(sub_ghz_radio PRIVATE
        settings_strs.cpp
        settings.cpp
    )
    target_include_directories(sub_ghz_radio PRIVATE
        .
    )
elseif (TARGET host_test OR TARGET host_test_fpu)
    foreach(_test_tgt IN ITEMS host_test host_test_fpu)
        if (TARGET ${_test_tgt})
            target_sources(${_test_tgt} PRIVATE
                settings_strs.cpp
                settings.cpp
                settings_migration.cpp
            )
            target_include_directories(${_test_tgt} PRIVATE
                .
            )
        endif()
    endforeach()
endif()
