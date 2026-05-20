add_executable(${PROJECT_NAME}_eskf main_eskf.cpp  
                               common_utility.cpp)
target_link_libraries(${PROJECT_NAME}_eskf 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.liw
                         ${PROJECT_NAME}.algo
                         ${third_party_libs}
                        )
add_dependencies(${PROJECT_NAME}_eskf ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_eskf_rosbag main_eskf_rosbag.cpp  
                        common_utility.cpp)
target_link_libraries(${PROJECT_NAME}_eskf_rosbag 
                  ${PROJECT_NAME}.common
                  ${PROJECT_NAME}.pre
                  ${PROJECT_NAME}.liw
                  ${PROJECT_NAME}.algo
                  ${PROJECT_NAME}.tools
                  ${third_party_libs}
                 )
add_dependencies(${PROJECT_NAME}_eskf_rosbag ${catkin_EXPORTED_TARGETS})



add_executable(test_analytic_factor test_analytic_factor.cpp )
target_link_libraries(test_analytic_factor
            ${PROJECT_NAME}.common
            ${PROJECT_NAME}.pre
            ${PROJECT_NAME}.liw
            ${PROJECT_NAME}.algo
            ${third_party_libs}
        )

add_executable(test_do_ct_diagnostics test_do_ct_diagnostics.cpp)
target_link_libraries(test_do_ct_diagnostics
            ${PROJECT_NAME}.common
            ${PROJECT_NAME}.liw
            ${third_party_libs}
        )

add_executable(test_cloud_convert_livox test_cloud_convert_livox.cpp)
target_link_libraries(test_cloud_convert_livox
            ${PROJECT_NAME}.pre
            ${third_party_libs}
        )
add_dependencies(test_cloud_convert_livox ${catkin_EXPORTED_TARGETS})
