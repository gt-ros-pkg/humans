#!/usr/bin/python
#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## Controlling Robotis Dynamixel RX-28 & RX-64 servos from python
## using the USB2Dynamixel adaptor.

## Authors: Travis Deyle & Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


# ROS imports
import roslib
roslib.load_manifest('robotis')
import rospy

from std_msgs.msg import Float64
from robotis.srv import None_Float
from robotis.srv import None_FloatResponse
from robotis.srv import MoveAng
from robotis.srv import MoveAngResponse
from robotis.srv import None_Int32
from robotis.srv import None_Int32Response

import robotis.lib_robotis as rs
import time
import math


class ROS_Robotis_Server():
    def __init__(self, servo = None, name = '' ):
        if servo == None:
            raise RuntimeError( 'ROS_Robotis_Servo: No servo specified for server.\n' )

        self.servo = servo
        self.name = name
        
        try:
            rospy.init_node( 'robotis_servo_' + self.name )
        except rospy.ROSException:
            pass

        print 'ROS_Robotis_Servo: Starting Server /robotis/servo_' + self.name
        self.channel = rospy.Publisher('/robotis/servo_' + self.name, Float64)

        self.__service_ang = rospy.Service('/robotis/servo_' + name + '_readangle',
                                           None_Float, self.__cb_readangle)

        self.__service_ismove = rospy.Service('/robotis/servo_' + name + '_ismoving',
                                              None_Int32, self.__cb_ismoving)

        self.__service_moveang = rospy.Service('/robotis/servo_' + name + '_moveangle',
                                               MoveAng, self.__cb_moveangle)

    def __cb_readangle( self, request ):
        ang = self.update_server()
        return None_FloatResponse( ang )

    def __cb_ismoving( self, request ):
        status = self.servo.is_moving()
        return None_Int32Response( int(status) )

    def __cb_moveangle( self, request ):
        ang = request.angle
        angvel = request.angvel
        blocking = bool( request.blocking )
        self.servo.move_angle( ang, angvel, blocking )
        return MoveAngResponse()

    def update_server(self):
        ang = self.servo.read_angle()
        self.channel.publish( Float64(ang) )
        return ang


class ROS_Robotis_Client():
    def __init__(self, name = '' ):
        self.name = name

        self.__service_ang = rospy.ServiceProxy('/robotis/servo_' + name + '_readangle',
                                                None_Float)

        self.__service_ismove = rospy.ServiceProxy('/robotis/servo_' + name + '_ismoving',
                                                   None_Int32)
        
        self.__service_moveang = rospy.ServiceProxy('/robotis/servo_' + name + '_moveangle',
                                               MoveAng)

    def read_angle( self ):
        resp = self.__service_ang()
        ang = resp.value
        return ang
        
    def is_moving( self ):
        resp = self.__service_ismove()
        return bool( resp.value )
        
    def move_angle( self, ang, angvel = math.radians(50), blocking = True ):
        self.__service_moveang( ang, angvel, int(blocking) )

if __name__ == '__main__':
    print 'Sample Server: '

    # Important note: You cannot (!) use the same device in another
    # process. The device is only "thread-safe" within the same
    # process (i.e.  between servos (and callbacks) instantiated
    # within that process) 
    
    dev_name = '/dev/robot/servo_left'
    ids = [11, 12]
    names = ['pan', 'tilt']

    dyn = rs.USB2Dynamixel_Device( dev_name )

    servos = [ rs.Robotis_Servo( dyn, i ) for i in ids ]
    ros_servers = [ ROS_Robotis_Server( s, n ) for s,n in zip( servos, names ) ]

    try:
        while not rospy.is_shutdown():
            [ s.update_server() for s in ros_servers ]
            time.sleep(0.001)
    except:
        pass

    for n in names:
        print 'ROS_Robotis_Servo: Shutting Down /robotis/servo_'+n


    
## SAMPLE CLIENTS:
        
#     tilt = ROS_Robotis_Client( 'tilt' )
#     tilt.move_angle( math.radians( 0 ), math.radians(10), blocking=False)
#     while tilt.is_moving():
#         print 'Tilt is moving'

#     pan = ROS_Robotis_Client( 'pan' )
#     pan.move_angle( math.radians( 0 ), math.radians(10), blocking=False)
#     while pan.is_moving():
#         print 'pan is moving'

