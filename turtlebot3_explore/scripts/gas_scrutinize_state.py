#!/usr/bin/env python
import smach
import rospy

#define scrutinize state
class ScrutinizeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'

# define move front state
class MoveFrontState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'

# define move back state
class MoveBackState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'

# define move turn state
class MoveTurnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'

# define stoped state
class MoveStopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'


class ExploredState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])
        self.dummy_bool = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(1)
        if self.dummy_bool:
            return 'done'
        else:
            return 'error'