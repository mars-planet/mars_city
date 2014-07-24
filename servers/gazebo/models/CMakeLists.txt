cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMMAND_CMAKE_POLICY)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMMAND_CMAKE_POLICY)

set(
	CMAKE_INSTALL_PREFIX /usr/share 
	CACHE_STRING "Install Path"
)

project(trevor_models)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

exec_program(date ARGS -u +%Y-%m-%DT%H:%M:%S OUTPUT_VARIABLE CURRENT_DATE)

set(dirs
	chain_curved
	chain_link
	chain_straight
	chassis
	gripper
	imu_mount
	stereo
	track_drive
	trevor
	ultrasonic
	wheel
)

set(
	MODEL_URI_LIST ""
	CACHE INTERNAL "Model URI List" FORCE)

# for special case of DCMAKE_INSTALL_PREFIX=~/.gazebo ..
#     fails due to local path, so convert local paths to absolute path
get_filename_component(CMAKE_INSTALL_PREFIX_ABSOLUTE ${CMAKE_INSTALL_PREFIX} ABSOLUTE)

foreach (dir ${dirs})
 	add_custom_target
  	(${dir} ALL COMMAND mkdir -p ${PROJECT_BINARY_DIR}/${dir}
    	COMMAND tar czvf ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz ../${dir}
    )

	install(DIRECTORY ${dir} DESTINATION ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models)
	install(FILES ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz DESTINATION
		${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/${dir})
		set(MODEL_URI_LIST "${MODEL_URI_LIST}<uri>file://${dir}</uri>\n"
	)
endforeach ()

configure_file(
	"${CMAKE_SOURCE_DIR}/manifest.xml.in"
    "${PROJECT_BINARY_DIR}/manifest.xml"
  )

install(
	FILES "${PROJECT_BINARY_DIR}/manifest.xml" DESTINATION
	${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/
)

configure_file(
	"${CMAKE_SOURCE_DIR}/database.config.in"
    "${PROJECT_BINARY_DIR}/database.config"
)

install(
	FILES "${PROJECT_BINARY_DIR}/database.config" DESTINATION
    ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/
)

message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models")

# This must always be last!
include(CPack)