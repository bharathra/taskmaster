#! /usr/bin/env python3

import json
from typing import List

import rospy
from std_msgs.msg import String, Bool

from action_graph import Agent, Action, State


class TaskMaster(Agent):

    def __init__(self, name=None) -> None:
        if name is None:
            name = self.__class__.__name__
        self.name = name

        self.cache: State = {}
        self.goals_q: List[State] = []

        self.plan_publisher = rospy.Publisher(f'/cor/{self.name}/action_graph/plan',
                                              String, latch=True, queue_size=1)
        #
        self.state_subscriber = rospy.Subscriber(f'cor/{self.name}/state',
                                                 String, self.state_received_cb, queue_size=100)
        self.goals_subcriber = rospy.Subscriber(f'cor/{self.name}/goal',
                                                String, self.goals_recived_cb, queue_size=100)
        #
        self.estop_subscriber = rospy.Subscriber(f'cor/{self.name}/estop', Bool,
                                                 self.estop_cb, queue_size=1, tcp_nodelay=True)
        self.reset_subscriber = rospy.Subscriber(f'cor/{self.name}/reset', Bool,
                                                 self.reset_cb, queue_size=1, tcp_nodelay=True)

    def auto_load_actions(self):
        self.load_actions([a(self) for a in Action.__subclasses__()])

    def state_received_cb(self, state_dict_str: String):
        state: State = json.loads(state_dict_str.data)
        self.state.update(state)
        rospy.loginfo(f'[{rospy.get_name()}] State updated. State: {state}')

    def goals_recived_cb(self, goal_dict_str: String):
        goal: State = json.loads(goal_dict_str.data)
        self.goals_q.append(goal)
        rospy.loginfo(f'[{rospy.get_name()}] Goal appended. Goal: {goal}')

    def estop(self):
        raise NotImplementedError()

    def estop_cb(self, flag: Bool = None):
        self.estop()
        self.abort()
        self.goals_q.clear()
        self.plan_publisher.publish(f'[{rospy.get_name()}] ABORTING EXECUTION!!')

    def reset_cb(self, flag: Bool = None):
        self.plan_publisher.publish(f'[{rospy.get_name()}] EXECUTION RESET.')
        self.reset()

    def get_plan_description(self, plan: List[Action]) -> str:
        description = ''
        for action in plan:
            description += f'0#{action.__class__.__name__};'
            for tag, goal in action.effects.items():
                description += f'1#{tag}:{str(goal)};'
        return description

    def fulfill_goals_in_q(self):
        #
        if not self.goals_q:
            return
        #
        try:
            goal: State = self.goals_q.pop(0)
            for plan in self.achieve_goal_interactive(goal):
                self.plan_publisher.publish(self.get_plan_description(plan))
            #
            self.plan_publisher.publish(f"GOAL FULFILLED: \n {goal}!")
            #
        except Exception as _ex:
            self.goals_q.clear()
            self.plan_publisher.publish(f"ERROR! {_ex}")
            rospy.sleep(1)
