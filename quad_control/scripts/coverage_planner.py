#!/usr/bin/env python
"""This module implements a ROS node that
acts as a planner for a coverage task."""



import utilities.coverage_utilities as cov
import geometry_msgs.msg as gms
import std_msgs.msg as sms
import quad_control.msg as qms
import quad_control.srv as qsv
import threading as thd
import numpy as np
import rospy as rp
import random as rdm





__NAME = rp.get_param('name', 'Axel')
__OTHERS_NAMES = rp.get_param('others_names', 'Bo Calle David').split()
__TOLERANCE = 0.2

__INITIAL_POSE = cov.INITIAL_POSES[__NAME]
__INITIAL_LANDMARKS = cov.INITIAL_LANDMARKS_LISTS[__NAME]
__COLOR = cov.AGENTS_COLORS[__NAME]
__agent = cov.Agent(*__INITIAL_POSE, landmarks=__INITIAL_LANDMARKS, color=__COLOR)

__lock = thd.Lock()
__token = rp.get_param('token', False)

# __landmarks = cov.INITIAL_LANDMARKS_LISTS[__NAME]
# __position = np.zeros(2)
# __versor = cov.versor_from_angle(0.0)

__possible_trade_partners = list(__OTHERS_NAMES)
__coverage_done_flag = False
    







def __trade_landmarks_handler(request):
    global __lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners
    __lock.acquire()
    others_landmarks = cov.landmarks_from_point_2d_array(request.landmarks)
    ox = request.pose.x
    oy = request.pose.y
    otheta = request.pose.theta
    old_landmarks = __agent.get_landmarks()
    success, i2r, l2a = __agent.trade(ox, oy, otheta, others_landmarks)
    assert len(__agent.get_landmarks())-len(old_landmarks) == len(i2r)-len(l2a)
    if success:
        __possible_trade_partners = list(__OTHERS_NAMES)
        #__landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
        #__landmarks_pub.publish(__agent.get_landmark_array())
    elif request.name in __possible_trade_partners:
        __possible_trade_partners.remove(request.name)
    __lock.release()
    array = cov.point_2d_array_from_landmarks(l2a)
    rp.logwarn(request.name + ' requests a trade with ' + __NAME)
    #rp.logwarn(request)
    rp.logwarn('Success of the trade: ' + str(success))
    return success, i2r, array


def __receive_token_handler(request):
    global __lock
    global __possible_trade_partners
    global __token
    __lock.acquire()
    if len(__possible_trade_partners) > 0:
        __token = True
        result = True
        rp.logwarn(__NAME + ': token accepted')
    else:
        result = False
        rp.logwarn(__NAME + ': token refused')
    __lock.release()
    return result
    
    
def __coverage_done_callback(msg):
    global __coverage_done_flag, __lock
    __lock.acquire()
    __coverage_done_flag = True
    __lock.release()

    
def __pose_callback(pose):
    global __lock
    global __agent
    __lock.acquire()
    __agent.set_pose(pose.x, pose.y, pose.theta)
    __lock.release()
    
    
    
rp.init_node('coverage_planner')
__INITIAL_TIME = rp.get_time()
__RATE = rp.Rate(1e2)

rp.Subscriber('pose_2d', gms.Pose2D, __pose_callback)
rp.Subscriber('/coverage_done', sms.Empty, __coverage_done_callback)
rp.Service('trade_landmarks', qsv.TradeLandmarks, __trade_landmarks_handler)
rp.Service('receive_token', qsv.ReceiveToken, __receive_token_handler)

__trade_proxies = {}
__token_proxies = {}
for name in __OTHERS_NAMES:
    rp.wait_for_service('/' + name + '/trade_landmarks')
    __trade_proxies[name] = rp.ServiceProxy('/' + name + '/trade_landmarks', qsv.TradeLandmarks)
    rp.wait_for_service('/' + name + '/receive_token')
    __token_proxies[name] = rp.ServiceProxy('/' + name + '/receive_token', qsv.ReceiveToken)

__cmd_vel_pub = rp.Publisher('cmd_vel', gms.Pose2D, queue_size=10)
__num_landmarks_pub = rp.Publisher('num_landmarks', sms.Int32, queue_size=10)
__cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)
__landmarks_pub = rp.Publisher('landmarks', qms.Point2DArray, queue_size=10)
__done_pub = rp.Publisher('/coverage_done', sms.Empty, queue_size=10)



def __work():
    global __lock
    global __agent
    global __NAME, __OTHERS_NAMES, __possible_trade_partners, __trade_proxies
    global __token
    global __cmd_vel_pub, __cov_pub, __landmarks_pub, __num_landmarks_pub
    global __INITIAL_TIME
    __lock.acquire()
    dot_p = __agent.position_coverage_gradient()
    dot_theta = __agent.orientation_coverage_gradient()
    coverage = __agent.coverage()
    if np.linalg.norm(dot_p) > 10.0:
        dot_p *= 10.0/np.linalg.norm(dot_p)
    dot_theta = np.clip(dot_theta, -10.0, 10.0)
    cmd_vel = gms.Pose2D(dot_p[0], dot_p[1], dot_theta)
    __cmd_vel_pub.publish(cmd_vel)
    __cov_pub.publish(coverage)
    __num_landmarks_pub.publish(len(__agent.get_landmarks()))
    if np.linalg.norm(dot_p) < __TOLERANCE and np.linalg.norm(dot_theta) < __TOLERANCE and __token:
        if len(__possible_trade_partners)>0:
            partner = rdm.choice(__possible_trade_partners)
            pose = gms.Pose2D(*__agent.get_pose())
            landmark_array = cov.point_2d_array_from_landmarks(__agent.get_landmarks())
            assert len(landmark_array.data) == len(__agent.get_landmarks())
            response = __trade_proxies[partner](__NAME, pose, landmark_array)
            if response.success:
                __possible_trade_partners = list(__OTHERS_NAMES)
                i2r = response.indexes_to_remove
                l2a = cov.landmarks_from_point_2d_array(response.landmarks_to_add)
                __agent.update_landmarks(i2r, l2a)
                __landmarks_pub.publish(cov.point_2d_array_from_landmarks(__agent.get_landmarks()))
                # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                #__agent.set_landmarks(new_landmarks)
                # assert len(__agent.get_landmarks()) == len(new_landmarks)
                # array = __agent.get_landmark_array()
                # assert len(array.data) == len(new_landmarks)
                #__landmarks_pub.publish(array)
            else:
                __possible_trade_partners.remove(partner)
                # new_landmarks = [cov.Landmark.from_pose2d(lmk) for lmk in response.landmarks.data]
                # __agent.set_landmarks(new_landmarks)
        token_accepted = False
        possible_token_receivers = list(__OTHERS_NAMES)
        while not token_accepted and len(possible_token_receivers)>0:
            rp.logwarn(__NAME + ': possible token receivers: ' + str(possible_token_receivers))
            name = rdm.choice(possible_token_receivers)
            possible_token_receivers.remove(name)
            rsp = __token_proxies[name]()
            token_accepted = rsp.accepted
        if token_accepted:
            __token = False
        elif len(__possible_trade_partners) == 0:
            final_time = rp.get_time()
            rp.logwarn('Finish! Elapsed time: ' + str(final_time-__INITIAL_TIME))
            __done_pub.publish(sms.Empty())
            __token = False
    __lock.release()
    
    


__lock.acquire()
while not rp.is_shutdown() and not __coverage_done_flag:
    __lock.release()
    __work()
    __RATE.sleep()
    __lock.acquire()
rp.logwarn(__NAME + ': Coverage is done!')
__lock.release()
