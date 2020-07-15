add_library( SLAM SHARED
    realsense.cpp
    slamBase.cpp
)

target_link_libraries( SLAM
    ${THIRD_PARTY_LIBS}
)
