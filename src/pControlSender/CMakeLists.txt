#--------------------------------------------------------
# The CMakeLists.txt for:                       pControlSender
# Author(s):                                   Tyler Paine
#--------------------------------------------------------

SET(SRC
  ControlSender.cpp
  ControlSenderCore.cpp
  ControlSender_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pControlSender ${SRC})

# bb_command provides the command envelope, CommandMailbox and the
# wire dialect -- the same parser and lease the arbiter uses.
TARGET_LINK_LIBRARIES(pControlSender
   bb_command
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)


