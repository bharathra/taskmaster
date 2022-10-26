#! /usr/bin/env python3

import os
import json
import yaml
from typing import List

import rospy
import rosparam
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

    def get_saved_state(self, filepath: str) -> State:
        #
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                yamlfile = yaml.load(f, Loader=yaml.FullLoader)
            rosparam.upload_params(f'cor/{self.name}/state', yamlfile)
            state: State = rospy.get_param(f'cor/{self.name}/state/', {})
            self.save_state(filepath)
            return state
            #
        raise Exception('Error loading robot state config')

    def save_state(self, filepath: str):
        rosparam.dump_params(filepath, f'cor/{self.name}/state')

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
        rospy.logerr(f'[{rospy.get_name()}] ABORTING EXECUTION!!')

    def reset_cb(self, flag: Bool = None):
        self.reset()
        self.plan_publisher.publish(f'[{rospy.get_name()}] EXECUTION RESET.')
        rospy.logwarn(f'[{rospy.get_name()}] EXECUTION RESET.')

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
            self.cache.clear()
            return
        #
        try:
            goal: State = self.goals_q.pop(0)
            for plan in self.achieve_goal_interactive(goal):
                self.plan_publisher.publish(self.get_plan_description(plan))
            #
            self.plan_publisher.publish(f"0#COMPLETED; 1#GOAL:{goal};;;")
            rospy.loginfo(f"COMPLETED: \n {goal}!")
            #
        except Exception as _ex:
            self.goals_q.clear()
            self.plan_publisher.publish(f"0#FAILED!; 1#{goal}:{_ex};;;")
            rospy.logerr(f"FAILED! {_ex}")
            rospy.sleep(1)
