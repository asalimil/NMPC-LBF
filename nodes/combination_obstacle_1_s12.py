#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import time
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates

class Combination_1():
    def __init__(self, t, dt):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.t = t; self.dt = dt
        self.moving()

    def moving(self):
        t = self.t
        dt = self.dt
        # while not rospy.is_shutdown():
        model = rospy.wait_for_message('gazebo/model_states', ModelStates)
        for i in range(len(model.name)):
            if model.name[i] == 'obstacle_1':
                obstacle_1 = ModelState()
                obstacle_1.model_name = model.name[i]
                obstacle_1.pose = model.pose[i]
                obstacle_1.twist = Twist()
                # obstacle_1.pose.position.x += 0.05
                # obstacle_1.pose.position.y += 0.0
                obstacle_1.pose.position.x = 1.5 - 1.0*np.sin(0.1*t + np.pi/2)
                obstacle_1.pose.position.y = 1.0
                obstacle_1.twist.linear.x = -0.1*np.cos(0.15*t + np.pi/2)
                obstacle_1.twist.linear.y = 0.0
                self.pub_model.publish(obstacle_1)
                self.obs1_x = obstacle_1.pose.position.x
                self.obs1_y = obstacle_1.pose.position.y
                self.obs1_vx = obstacle_1.twist.linear.x
                self.obs1_vy = obstacle_1.twist.linear.y
                # t = t + dt
                # time.sleep(dt)
        # return self.obs1_x, self.obs1_y                
        return self.obs1_x, self.obs1_y, self.obs1_vx, self.obs1_vy


