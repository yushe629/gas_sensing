#!/usr/bin/env python
import rospy
import smach
import smach_ros
from gas_patrol_state import GasPatrolState
from gas_roughly_search_state import RoughlySearchState

def main():
    rospy.init_node('gas_explore_smach')

    sm_top = smach.StateMachine(outcomes=['completed', 'failure'])
    with sm_top:
        # wip: check 1 State
        # smach.StateMachine.add('GasPatrolState', GasPatrolState(), transitions={'completed': 'RoughlySearchState', 'continue': 'GasPatrolState', 'error': 'failure'})
        smach.StateMachine.add('RoughlySearchState', RoughlySearchState(), transitions={'completed': 'completed', 'continue': 'RoughlySearchState',  'error': 'failure'})
        # smach.StateMachine.add('ScrutinizeState', ScrutinizeState(), transitions={'done': 'completed', 'error': 'failure'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass