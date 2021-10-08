#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys, getopt, math


class cmdvel2gazebo:

    def __init__(self,car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius):
        
        rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.pub_steerL = rospy.Publisher('/rbcar/left_steering_joint_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/rbcar/right_steering_joint_position_controller/command', Float64, queue_size=1)

        self.pub_rearL = rospy.Publisher('/rbcar/left_rear_axle_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/rbcar/right_rear_axle_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/rbcar/left_front_axle_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/rbcar/right_front_axle_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0
        
        self.L = car_wheel_base
        self.T=car_wheel_threat
        self.wheel_radius = wheel_radius

        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside=max_abs_steer
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rIdeal = self.L/math.tan(self.maxsteerInside)
        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        self.rMin = rIdeal+(self.T/2.0)

        self._check_cmd_vel_ready()

    def _check_cmd_vel_ready(self):
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message('/cmd_vel', Twist, timeout=1.0)
                self.process_cmd_vel_data(data)
                rospy.logdebug("Current cmd_vel READY=>")

            except:
                rospy.logwarn("Current cmd_vel not ready yet, retrying...")

        
    def process_cmd_vel_data(self,data):
        self.linear_velocity = data.linear.x        
        # We limit the minimum value of the Steering Radius
        # Todo Process negatives
        if data.angular.z != 0.0:
            steering_radius_raw = abs(self.linear_velocity / data.angular.z)
            self.steering_radius = max(abs(steering_radius_raw), self.rMin)
            # We consider that turning left should be positive
            self.turning_sign = -1 * math.copysign(1,data.angular.z)
            self.omega_turning_speed = self.linear_velocity / self.steering_radius
        else:
            self.steering_radius = -1
            self.turning_sign = 0.0
            self.omega_turning_speed = 0.0
        
        self.lastMsg = rospy.Time.now()
    
    def callback(self,data):
        """
        We get the linear velocity and the desired Turning Angurlar velocity.
        We have to convert it to Turning Radius
        """
        self.process_cmd_vel_data(data)


    def publish(self):

        # delta_last_msg_time = rospy.Time.now() - self.lastMsg

        # msgs_too_old = delta_last_msg_time > self.timeout
        # if msgs_too_old:
        #     msgRearFront = Float64()
        #     msgFrontSteer = Float64()

        #     self.pub_rearL.publish(msgRearFront)
        #     self.pub_rearR.publish(msgRearFront)
            
        #     self.pub_steerR.publish(msgRearFront)
        #     self.pub_steerL.publish(msgRearFront)

        #     self.pub_steerR.publish(msgFrontSteer)
        #     self.pub_steerL.publish(msgFrontSteer)

        #     rospy.logerr("Message Too OLD")

        if self.steering_radius >= 0:

            rospy.logdebug("TURNING")

            msgRearR = Float64()
            msgRearL = Float64()

            auxi = (self.steering_radius +(  ( -1 * self.turning_sign)*(self.T / 2.0) )  ) 
            linear_vel_interior = self.omega_turning_speed * auxi

            auxe = (self.steering_radius +(  ( 1 * self.turning_sign)*(self.T / 2.0) )  ) 
            linear_vel_exterior = self.omega_turning_speed * auxe

            wheel_turnig_speed_interior = linear_vel_interior / self.wheel_radius
            wheel_turnig_speed_exterior = linear_vel_exterior / self.wheel_radius
            
            msgRearR.data = wheel_turnig_speed_interior
            msgRearL.data = wheel_turnig_speed_exterior            

            #### Front Steering Wheels

            msgFrontR = Float64()
            msgFrontL = Float64()

            
            distance_to_turning_point_interior = math.sqrt(pow(self.L,2)+pow(auxi,2))

            
            distance_to_turning_point_exterior = math.sqrt(pow(self.L,2)+pow(auxe,2))

            linear_vel_front_interior = self.omega_turning_speed * distance_to_turning_point_interior
            linear_vel_front_exterior = self.omega_turning_speed * distance_to_turning_point_exterior

            wheel_turnig_speed_front_interior = linear_vel_front_interior / self.wheel_radius
            wheel_turnig_speed_front_exterior = linear_vel_front_exterior / self.wheel_radius

            msgFrontL.data = wheel_turnig_speed_front_exterior
            msgFrontR.data = wheel_turnig_speed_front_interior


            #### Steering Position

            msgFrontSteerR = Float64()
            msgFrontSteerL = Float64()

            alfa_interior = math.atan(self.L / auxi)
            alfa_exterior = math.atan(self.L / auxe)

            msgFrontSteerR.data = -1 * self.turning_sign * alfa_interior
            msgFrontSteerL.data = -1 * self.turning_sign * alfa_exterior


            #### We publish all the messages

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)
            
            self.pub_frontR.publish(msgFrontR)
            self.pub_frontL.publish(msgFrontL)

            self.pub_steerR.publish(msgFrontSteerR)
            self.pub_steerL.publish(msgFrontSteerL)

        else:
            rospy.logdebug("NOT TURNING")
            # if we aren't turning, everything is easy!
            msgRearFront = Float64()
            msgFrontSteer = Float64()
            msgRearFront.data = self.linear_velocity / self.wheel_radius
            msgFrontSteer.data = 0.0

            self.pub_rearL.publish(msgRearFront)
            self.pub_rearR.publish(msgRearFront)
            
            self.pub_frontR.publish(msgRearFront)
            self.pub_frontL.publish(msgRearFront)

            self.pub_steerR.publish(msgFrontSteer)
            self.pub_steerL.publish(msgFrontSteer)


def main():
    # we eventually get the ns (namespace) from the ROS parameter server for this node
    rospy.init_node('cmdvel2gazebo', anonymous=True, log_level=rospy.DEBUG)

    # Distance from Front to Rear axel
    car_wheel_base = 1.82278

    # Distance from left to right wheels
    car_wheel_threat = 1.02996

    # Calculated as the  maximum steering angle the inner wheel can do
    max_abs_steer = 0.7

    wheel_radius = 0.634 / 2.0
    node = cmdvel2gazebo(car_wheel_base, car_wheel_threat, max_abs_steer, wheel_radius)
    rate = rospy.Rate(10) # run at 10Hz
    while not rospy.is_shutdown():
        node.publish()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn("Reseted Time...")
            pass

if __name__ == '__main__':
    main()
    
