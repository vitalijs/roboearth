cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-codec-1.4.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-logging-1.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-cache-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpcore-4.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpmime-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi/owlapi-bin.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/hermit/HermiT.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jgrapht/jgrapht-jdk1.6.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/mysql/mysql-connector-java-5.1.13-bin.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/argo/argo-2.8.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/vecmath/vecmath.jar)
add_classpath(${PROJECT_SOURCE_DIR}/bin)

add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-codec-1.4.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-logging-1.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-cache-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpcore-4.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpmime-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi/owlapi-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/hermit/HermiT.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jgrapht/jgrapht-jdk1.6.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/mysql/mysql-connector-java-5.1.13-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/argo/argo-2.8.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/vecmath/vecmath.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/bin)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
rospack_add_java_executable(run roboearth.wp5.ReCommCore)

