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
import numpy as np
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates

class Combination_2():
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
            if model.name[i] == 'obstacle_2':
                obstacle_2 = ModelState()
                obstacle_2.model_name = model.name[i]
                obstacle_2.pose = model.pose[i]
                obstacle_2.twist = Twist()
                obstacle_2.pose.position.x = 1.5 - 1.0*np.sin(0.1*t)
                obstacle_2.pose.position.y = 0.5
                obstacle_2.twist.linear.x = -0.1*np.cos(0.1*t)
                obstacle_2.twist.linear.y = 0.0
                self.pub_model.publish(obstacle_2)
                self.obs2_x = obstacle_2.pose.position.x
                self.obs2_y = obstacle_2.pose.position.y
                self.obs2_vx = obstacle_2.twist.linear.x
                self.obs2_vy = obstacle_2.twist.linear.y
                # t = t + dt
                # time.sleep(dt)
        # return self.obs2_x, self.obs2_y                        
        return self.obs2_x, self.obs2_y, self.obs2_vx, self.obs2_vy

