cmake_minimum_required(VERSION 3.0.2)
project(michael_msgs)

set(MSG_DEPS
  std_msgs
  actionlib_msgs
  geometry_msgs
  trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

set(MSG_FILES
OmplRequest.msg
OmplResponse.msg
)

set(SRV_FILES
GetOmplResults.srv
)

# set(ACT_FILES
# ExecuteTrajectory.action
# MoveGroup.action
# MoveGroupSequence.action
# Pickup.action
# Place.action
# )

# add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
