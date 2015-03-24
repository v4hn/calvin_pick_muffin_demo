cmake_minimum_required(VERSION 2.8)
project(calvin_pick_muffin_demo)

find_package(catkin REQUIRED)

catkin_package()

install(FILES scripts/pick_muffin_demo.py RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
