#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import apriltag
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os 
import time
import numpy as np
import math

HOME_PATH = os.environ['HOME']
IMAGE_PATH = HOME_PATH + "/catkin_ws/local/images/dep_images/"
# if not os.path.exists(IMAGE_PATH):
#     os.makedirs(IMAGE_PATH)
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}
_NEXT_AXIS = [1, 2, 0, 1]

class ImageProcessing:
    def __init__(self):
        rospy.init_node('get_dep_cam_image',anonymous=True)
        rospy.Subscriber("/camera/color/image_raw",Image,self.get_image)
        rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.get_camera_info)
        self.bridge = CvBridge()
        self.cv_image = None

    def get_image(self, data):
        try:
            # self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height,data.width,-1)
            # print(cv_image)
        except CvBridgeError as e:
            print(e)

    def get_camera_info(self, data):
        self.camera_params = [data.K[0],data.K[4],data.K[2],data.K[5]] #640*360
        # self.camera_params = [475.0572671879335, 475.81470954056243, 324.1924887352527, 242.87098204691128] #640*480


    def rot2eul(self,R):
        beta = -np.arcsin(R[2,0])
        alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
        gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
        return np.array((alpha, beta, gamma))

    def save_image(self,image_name):
        if self.cv_image is not None:
            # print("{}{}".format(IMAGE_PATH,image_name))
            # print(self.cv_image)
            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            cv2.imwrite("{}{}".format(IMAGE_PATH,image_name),self.cv_image)
            return True
        else:
            print("cv_image is None!")
            return False
    def euler_matrix(self, ai, aj, ak, axes='sxyz'):
        """Return homogeneous rotation matrix from Euler angles and axis sequence.

        ai, aj, ak : Euler's roll, pitch and yaw angles
        axes : One of 24 axis sequences as string or encoded tuple

        >>> R = euler_matrix(1, 2, 3, 'syxz')
        >>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
        True
        >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
        >>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
        True
        >>> ai, aj, ak = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
        >>> for axes in _AXES2TUPLE.keys():
        ...    R = euler_matrix(ai, aj, ak, axes)
        >>> for axes in _TUPLE2AXES.keys():
        ...    R = euler_matrix(ai, aj, ak, axes)

        """
        
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]

        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]

        if frame:
            ai, ak = ak, ai
        if parity:
            ai, aj, ak = -ai, -aj, -ak

        si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
        ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
        cc, cs = ci*ck, ci*sk
        sc, ss = si*ck, si*sk

        M = np.identity(4)
        if repetition:
            M[i, i] = cj
            M[i, j] = sj*si
            M[i, k] = sj*ci
            M[j, i] = sj*sk
            M[j, j] = -cj*ss+cc
            M[j, k] = -cj*cs-sc
            M[k, i] = -sj*ck
            M[k, j] = cj*sc+cs
            M[k, k] = cj*cc-ss
        else:
            M[i, i] = cj*ck
            M[i, j] = sj*sc-cs
            M[i, k] = sj*cc+ss
            M[j, i] = cj*sk
            M[j, j] = sj*ss+cc
            M[j, k] = sj*cs-sc
            M[k, i] = -sj
            M[k, j] = cj*si
            M[k, k] = cj*ci
        return M
    def concatenate_matrices(self, *matrices):
        """Return concatenation of series of transformation matrices.

        >>> M = numpy.random.rand(16).reshape((4, 4)) - 0.5
        >>> numpy.allclose(M, concatenate_matrices(M))
        True
        >>> numpy.allclose(numpy.dot(M, M.T), concatenate_matrices(M, M.T))
        True

        """
        M = np.identity(4)
        for i in matrices:
            M = np.dot(M, i)
        return M
    def convert_to_4x4(self, matrix_R, matrix_T=[0,0,0]):
        new_matrix = np.zeros((4,4))
        new_matrix[:3,:3] = matrix_R
        new_matrix[:3,3] = matrix_T
        new_matrix[3,3] = 1
        return new_matrix

    def get_tf_from_apriltag(self, image_name):
        detector = apriltag.Detector()
        print("---------------------------------------------------------")
        try:
            img = cv2.imread("{}{}".format(IMAGE_PATH,image_name), cv2.IMREAD_GRAYSCALE)
            result = detector.detect(img)
            for _result in result:
                pose_result = detector.detection_pose(_result,self.camera_params,tag_size = 0.05) #tag_size = 0.039
                tag_rotation = pose_result[0][:3,:3]
                result_R = self.rot2eul(tag_rotation)
                result_T = pose_result[0][:3,3]
                print(result_R)
                camera_base_to_base = self.concatenate_matrices(self.euler_matrix(np.pi, 0, -np.pi/2),self.convert_to_4x4(np.linalg.inv(tag_rotation)),np.linalg.inv(self.euler_matrix(-np.pi/2, 0, -np.pi/2)))
                euler_camera_base_to_base = self.rot2eul(camera_base_to_base)


                print("euler_camera_base_to_base:{}".format((math.degrees(euler_camera_base_to_base[0]),math.degrees(euler_camera_base_to_base[1]),math.degrees(euler_camera_base_to_base[2]))))
                Tc_a = pose_result[0]
                T = self.concatenate_matrices(camera_base_to_base, self.euler_matrix(-np.pi/2, 0, -np.pi/2), Tc_a)
                translation_camera_base_to_base = [0.3, 0, -T[2,3]]
                print("depth_camera_joint rpy:{}".format(euler_camera_base_to_base))  #tf: depth_camera_joint  
                print("depth_camera_joint xyz:{}".format([0.3, 0, -T[2,3]]))  #tf: depth_camera_joint  
            return euler_camera_base_to_base, translation_camera_base_to_base

        except Exception as e:
            print(e)
        
    def compute_the_average(self, R_list, T_list):
        roll = sum([R[0] for R in R_list])/len(R_list)
        pitch = sum([R[1] for R in R_list])/len(R_list)
        yaw = sum([R[2] for R in R_list])/len(R_list)
        x = sum([T[0] for T in T_list])/len(T_list)
        y = sum([T[1] for T in T_list])/len(T_list)
        z = sum([T[2] for T in T_list])/len(T_list)
        return (roll, pitch, yaw), (x, y, z)

if __name__ == '__main__':
    image_process = ImageProcessing()
    image_name = ""
    a = 0
    euler_camera_base_to_base_list = []
    translation_camera_base_to_base_list = []
    while not rospy.is_shutdown():
        if a >= 10:
            print(image_process.compute_the_average(euler_camera_base_to_base_list,translation_camera_base_to_base_list))
            break
        else:
            image_name = "{}.jpg".format(a)
            res = image_process.save_image(image_name)
            time.sleep(0.5)
            if res is True:
                euler_camera_base_to_base, translation_camera_base_to_base = image_process.get_tf_from_apriltag(image_name)
                a+=1
                euler_camera_base_to_base_list.append(euler_camera_base_to_base)
                translation_camera_base_to_base_list.append(translation_camera_base_to_base)
                res = False

        time.sleep(0.5)
    rospy.spin()