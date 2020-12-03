#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "test.py"
__version__  = "2020.12"
__desc__     = "Test the FK/IK service"

"""
~~~ Developmnent Plan ~~~
[ ] Step 1
[ ] Step 2
"""

# === Init Environment =====================================================================================================================
# ~~~ Prepare Paths ~~~
import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
PARENTDIR = os.path.dirname( SOURCEDIR )
# ~~ Path Utilities ~~
def prepend_dir_to_path( pathName ): sys.path.insert( 0 , pathName ) # Might need this to fetch a lib in a parent directory

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi , sqrt , sin , cos
from random import random

# ~~ Special ~~
import numpy as np
import rospy
from std_msgs.msg import String , Float32MultiArray , Bool
from geometry_msgs.msg import PoseStamped, Point, Twist

# ~~ Local ~~
from ur_fk_ik.srv import FK , IK

def load_arr_to_pose( rspArr ):
    """ Reshape 1x16 array into 4x4 homogeneous matrix """
    _DEBUG = 0
    dimLen = 4
    rtnHomog = np.zeros( (4,4) )
    i = j = 0
    for elem in rspArr:
        if _DEBUG:  print "[" + str(j) + "," + str(i) + "]:" , elem
        rtnHomog[j,i] = elem
        i += 1
        if i == dimLen:
            i = 0
            j = (j+1) % dimLen
    return rtnHomog

def load_pose_to_arr( rspArr ):
    """ Reshape 4x4 homogeneous matrix into 1x16 array """
    _DEBUG = 0
    dimLen = 4
    i = j = 0
    rtnArr = []
    for k in range(16):
        elem = rspArr[j,i]
        if _DEBUG:  print "[" + str(j) + "," + str(i) + "]:" , elem
        rtnArr.append( elem ) 
        i += 1
        if i == dimLen:
            i = 0
            j = (j+1) % dimLen
    return rtnArr

def rand_lo_hi( lo , hi ):
    return random() * ( hi - lo ) + lo

def load_IK_request_arr( poseArr , q_seed ):
    # poseArr = load_pose_to_arr( poseMatx )
    poseVec = poseArr[:]
    for q_i in q_seed:
        poseArr.append( q_i )
    return poseArr

class FK_IK_Tester:
    """ Test the MoveIt Services """

    def __init__( self ):
        """ Set up vars and publishers """

        rospy.init_node( 'FK_IK_Tester' )
        
        rospy.wait_for_service( 'UR_FK_IK/FKsrv' )
        print "FK AVAILABLE"
        self.FKsrv = rospy.ServiceProxy( 'UR_FK_IK/FKsrv' , FK )

        rospy.wait_for_service( 'UR_FK_IK/IKsrv' )
        print "IK AVAILABLE"
        self.IKsrv = rospy.ServiceProxy( 'UR_FK_IK/IKsrv' , IK )

    def test1( self ):
        """ Test the FK and IK services """

        # Example Usage: Forward Kinematics
        # req = Float32MultiArray()
        req = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
        rsp = self.FKsrv( req )
        rsp = load_arr_to_pose( rsp.poseMatx )
        print rsp

    def test2( self ):

        wins   =   0
        Nrpt   = 200
        errTot =   0

        for i in range( Nrpt ):

            print "Iteration" , i+1
            factor = 0.5
            FKreq = [ rand_lo_hi( -pi*factor , +pi*factor ) for i in range(6) ]
            pose  = self.FKsrv( FKreq ).poseMatx
            print type( pose ) , pose
            FKrsp = load_arr_to_pose( pose )
            print "Request Joints:" , FKreq
            print "Request Pose:\n" , FKrsp

            # Example Usage: Inverse Kinematics
            # IKreq = Float32MultiArray()
            IKpose = load_pose_to_arr( FKrsp )
            if 0:
                IKseed = [ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]
            else:
                limit = pi/8
                IKseed = np.add( FKreq , [ rand_lo_hi( -limit , +limit ) for i in range(6) ] )
            IKreq = load_IK_request_arr( IKpose , IKseed )
            print "IK Request: " ,  IKreq
            IKrsp = self.IKsrv( IKreq )
            print "IK Response:" ,  IKrsp
            # print dir( IKrsp )
            if IKrsp.q_joints_valid[6] > 0:
                wins += 1

                FKchk = IKrsp.q_joints_valid[:6]
                FKcrs = load_arr_to_pose( self.FKsrv( FKchk ).poseMatx )
                print "\tJoint Differene:" , np.linalg.norm( np.subtract( np.array( FKreq ) , np.array( IKrsp.q_joints_valid[:6] ) ) )
                linDiff = np.linalg.norm( np.subtract( FKrsp[0:3,3] , FKcrs[0:3,3] ) )
                print "\tLinear Difference:" , linDiff
                errTot += linDiff

            print "Got IK response: " , IKrsp.q_joints_valid[:6] , "is it valid?:" , IKrsp.q_joints_valid[6]
            print "\n"

        print wins , "/" , Nrpt , "valid answers"
        print errTot / wins , "average linear error across valid answers"

if __name__ == "__main__":
    node = FK_IK_Tester()
    node.test1()
    print
    print
    node.test2()