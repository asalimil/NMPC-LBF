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
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates

class Combination():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):
        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'obstacle_2':
                    obstacle_2 = ModelState()
                    obstacle_2.model_name = model.name[i]
                    obstacle_2.pose = model.pose[i]
                    obstacle_2.twist = Twist()
                    obstacle_2.twist.linear.x = -0.2
                    obstacle_2.twist.linear.y = -0.2
                    self.pub_model.publish(obstacle_2)
                    time.sleep(0.1)

def main():
    rospy.init_node('combination_obstacle_2')
    try:
        combination = Combination()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
