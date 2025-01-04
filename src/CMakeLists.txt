
set (control_sources
        Frame.cpp
        Dataset.cpp
        Pipeline.cpp
        utility.cpp
        MotionTracker.cpp
)

add_library( gcc_ransac ${control_sources} )

#> Linking third-party libraries
target_link_libraries(gcc_ransac
        ${THIRD_PARTY_LIBS})

