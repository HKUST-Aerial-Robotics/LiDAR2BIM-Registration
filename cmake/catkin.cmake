find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        roslib
        std_msgs
        geometry_msgs
        message_generation
        sensor_msgs
        nav_msgs
        pcl_ros
        cv_bridge
        eigen_conversions
        )
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
list(APPEND ALL_TARGET_LIBRARIES ${catkin_LIBRARIES})
