add_executable( run_vo run_vo.cpp )
add_executable( test test.cpp )

target_link_libraries( run_vo SLAM )
target_link_libraries( test SLAM )