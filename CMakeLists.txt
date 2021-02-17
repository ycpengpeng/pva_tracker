cmake_minimum_required(VERSION 2.8.3)

project(pva_tracker)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-std=c++14 -g -O1 -ftree-vectorize -march=native -pthread)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	dynamic_reconfigure
	message_generation
	)

generate_dynamic_reconfigure_options(
		cfg/pid.cfg
		cfg/pid_ground.cfg
)
add_message_files(
		FILES
		input.msg
)
generate_messages(
		DEPENDENCIES
		std_msgs
)
catkin_package(
		CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  "../../devel/include"

	include/plan_zuan_quan
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(roslaunch)



add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES})
add_dependencies(tracker ${PROJECT_NAME}_gencfg)

add_executable(sim_control_test src/sim_control_test.cpp)
target_link_libraries(sim_control_test ${catkin_LIBRARIES})

add_executable(real_control_test src/real_control_test.cpp)
target_link_libraries(real_control_test ${catkin_LIBRARIES})

add_executable(hover src/hover.cpp)
target_link_libraries(hover ${catkin_LIBRARIES})


add_executable(plan_zuan_quan src/plan_zuan_quan.cpp)
target_link_libraries(plan_zuan_quan ${catkin_LIBRARIES})

add_executable(pub_zuan_quan src/pub_zuan_quan.cpp)
target_link_libraries(pub_zuan_quan ${catkin_LIBRARIES})

add_executable(test_take_off src/test_take_off.cpp)
target_link_libraries(test_take_off ${catkin_LIBRARIES})

add_executable(test_take_off_on_up src/test_take_off_on_up.cpp)
target_link_libraries(test_take_off_on_up ${catkin_LIBRARIES})

add_executable(hover_test_on_up src/hover_test_on_up.cpp)
target_link_libraries(hover_test_on_up ${catkin_LIBRARIES})

add_executable(plot_pva src/plot_pva.cpp)
target_link_libraries(plot_pva ${catkin_LIBRARIES})

add_executable(mpc_record src/mpc_record.cpp)
target_link_libraries(mpc_record ${catkin_LIBRARIES})

#[[add_executable(tiny_dnn_test1 src/tiny_dnn_test1.cpp)
target_link_libraries(tiny_dnn_test1 ${catkin_LIBRARIES})
target_include_directories(tiny_dnn_test1   PRIVATE /home/pengpeng/tiny-dnn)
set_target_properties(tiny_dnn_test1 PROPERTIES
		COMPILE_FLAGS "-pthread"
		LINK_FLAGS "-pthread")]]

add_executable(tiny_dnn_test2 src/tiny_dnn_test2.cpp)
target_link_libraries(tiny_dnn_test2 ${catkin_LIBRARIES})
target_include_directories(tiny_dnn_test2   PRIVATE /home/pengpeng/tiny-dnn)
set_target_properties(tiny_dnn_test2 PROPERTIES
		COMPILE_FLAGS "-pthread"
		LINK_FLAGS "-pthread")

#[[add_executable(mpc_tracker src/mpc_tracker.cpp)
target_link_libraries(mpc_tracker ${catkin_LIBRARIES})
target_include_directories(mpc_tracker   PRIVATE /home/pengpeng/tiny-dnn)
set_target_properties(mpc_tracker PROPERTIES
		COMPILE_FLAGS "-pthread"
		LINK_FLAGS "-pthread")]]

add_executable(mpc_tracker_pytorch src/mpc_tracker_pytorch.cpp)
target_link_libraries(mpc_tracker_pytorch ${catkin_LIBRARIES})
set(Torch_DIR /home/pengpeng/Downloads/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
target_link_libraries(mpc_tracker_pytorch ${TORCH_LIBRARIES} )


add_executable(random_fly src/random_fly.cpp)
target_link_libraries(random_fly ${catkin_LIBRARIES})

add_executable(build_dataset src/build_dataset.cpp)
target_link_libraries(build_dataset ${catkin_LIBRARIES})
target_include_directories(build_dataset   PRIVATE /home/pengpeng/tiny-dnn)
set_target_properties(build_dataset PROPERTIES
		COMPILE_FLAGS "-pthread"
		LINK_FLAGS "-pthread")

add_executable(square_fly src/square_fly.cpp)
target_link_libraries(square_fly ${catkin_LIBRARIES})


add_executable(record_plot_data src/record_plot_data.cpp)
target_link_libraries(record_plot_data ${catkin_LIBRARIES})

add_executable(fake_random_fly src/fake_random_fly.cpp)
target_link_libraries(fake_random_fly ${catkin_LIBRARIES})