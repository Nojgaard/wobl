set(SCRIPT_EXECUTABLES
	imu_calibrate
	imu_node
	servo_midpoint
	servo_node
	servo_ping
	servo_set_id
	ddsm_ping
	ddsm_test
	ddsm_set_id
	test_publisher
	test_subscriber
)

foreach(EXE ${SCRIPT_EXECUTABLES})
	add_executable(${EXE} ${EXE}.cpp)
	target_link_libraries(${EXE} PRIVATE woblcpp)
endforeach()
