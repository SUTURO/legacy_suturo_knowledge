cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# ROS packages
rosbuild_find_ros_package(json_prolog)
#rosbuild_find_ros_package(suturo_knowledge_ps_export)
rosbuild_find_ros_package(ias_knowledge_base)
rosbuild_find_ros_package(knowrob_objects)
rosbuild_find_ros_package(jpl)
rosbuild_find_ros_package(rosjava_jni)
rosbuild_find_ros_package(tfjava)

# Java-Prolog bridge
add_jni_path(${jpl_PACKAGE_PATH}/prolog)
add_ld_preload(/usr/lib/libswipl.so)
add_ld_preload(/usr/lib/libpl.so)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)
add_classpath(${tfjava_PACKAGE_PATH}/bin)
#add_classpath(${suturo_knowledge_ps_export_PACKAGE_PATH}/bin)

# build java
add_runtime_classpath(${PROJECT_SOURCE_DIR}/bin)
add_java_source_dir( ${PROJECT_SOURCE_DIR}/src/java )
#rospack_add_java_executable( DummyJava de.suturo.knowledge.foodreasoner.PrologBridge )

# replace path to local owl files
set( OWL_PATH_PREFIX ${suturo_knowledge_foodreasoner_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)
set( KNOWROB_PATH_PREFIX ${knowrob_objects_PACKAGE_PATH} )
fix_ros_destination_path(KNOWROB_PATH_PREFIX)


file(GLOB inputs owl/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

# replace path to local prolog files
rosbuild_find_ros_package(suturo_knowledge_foodreasoner)
set( LOCAL_PACKAGE_PATH ${suturo_knowledge_foodreasoner_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
