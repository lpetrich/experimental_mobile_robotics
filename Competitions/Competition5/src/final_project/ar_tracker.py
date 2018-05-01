#!/usr/bin/env python
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2, cv_bridge
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, LaserScan
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist, Pose
from image_geometry import PinholeCameraModel
import math
import time
import tf

class ARTracker:
    def __init__(self):
    	cv2.namedWindow('AR_TRACKER')
        self.bridge = cv_bridge.CvBridge()
        self.cam_info_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.info_cb)
        # self.pose_sub = rospy.Subscriber('pose', Pose, self.pose_cb)
        # self.img_sub = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.img_cb, queue_size = 1, buff_size=2**24)
        self.img_sub = rospy.Subscriber('usb_cam/image_raw/compressed', CompressedImage, self.img_cb, queue_size = 1, buff_size=2**24)
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.pub = rospy.Publisher('ar_location', String, queue_size=5)
        # self.control_pub = rospy.Publisher('ar_control', Bool, queue_size=5)
        # self.stop_linear = False
        # self.straight_ahead = False
        # self.init = [False, False]
        # self.initial_theta = None
        # self.theta = None
        # self.pos = False
        self.count = 0
        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3)
        self.objp = np.zeros((6*8, 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
        # self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.K = None
        self.D = None
        self.markers = []
        self.cam_info = None
        self.pinhole = PinholeCameraModel()

    def info_cb(self, msg):
        self.cam_info = msg
        self.K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D)

    # def pose_cb(self, msg):
    #     if not self.init[0]:
    #         self.initial_theta = np.degrees(msg.orientation.z)
    #         self.init[0] = True
    #     else:
    #         self.theta = np.degrees(msg.orientation.z)

    def ar_cb(self,msg):
        self.markers = msg.markers
        print self.markers

    # def qv_mult(self, q1, v1):
    #     v1 = tf.transformations.unit_vector(v1)
    #     q2 = list(v1)
    #     q2.append(0.0)
    #     return tf.transformations.quaternion_multiply( tf.transformations.quaternion_multiply(q1, q2), tf.transformations.quaternion_conjugate(q1) )[:3]

    def img_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if len(self.markers) > 0 and self.cam_info != None:
            print 'found ar tag'
            self.count += 1
            camera = self.pinhole.fromCameraInfo(self.cam_info)
            mid_list = []
            corners = []
            for i in range(len(self.markers)):
                tempCorner = self.pinhole.project3dToPixel([self.markers[i].pose.pose.position.x, self.markers[i].pose.pose.position.y, self.markers[i].pose.pose.position.z])
                tmp = []
                corner = [tempCorner[0], tempCorner[1]]
                circle = [int(tempCorner[0]), int(tempCorner[1])]
                tmp.append(np.array(corner, dtype="float32"))
                mid_list.append(np.array(tmp, dtype="float32"))
                corners.append(circle)
                cv2.circle(frame, tuple(circle), 3, (100,100,100), 5)
            px = self.markers[0].pose.pose.position.x
            py = self.markers[0].pose.pose.position.y
            pz = self.markers[0].pose.pose.position.z
            ox = self.markers[0].pose.pose.orientation.x
            oy = self.markers[0].pose.pose.orientation.y
            oz = -self.markers[0].pose.pose.orientation.z
            ow = -self.markers[0].pose.pose.orientation.w
            tvecs = np.array([px, py, pz])
            angle = 2 * math.acos(ow)
            x = ox / math.sqrt(1 - ow*ow)
            y = oy / math.sqrt(1 - ow*ow)
            z = oz / math.sqrt(1 - ow*ow)
            ratio = math.sqrt(x*x + y*y + z*z)
            #normalize them
            x = x / ratio*angle
            y = y / ratio*angle
            z = z / ratio*angle
            rvecs = np.array([x, y, z])
            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
            origin = tuple(imgpts[3].ravel())
            X = tuple(imgpts[0].ravel())
            Y = tuple(imgpts[1].ravel())
            Z = tuple(imgpts[2].ravel())
            pos_msg = str(origin) + str(X) + str(Y) + str(Z)
            if self.count > 15:
                self.pub.publish(pos_msg)
            image = cv2.line(frame, origin, X, (255,0,0), 5)
            image = cv2.line(frame, origin, Y, (0,255,0), 5)
            image = cv2.line(frame, origin, Z, (0,0,255), 5)
            cv2.imshow('AR_TRACKER',frame)
        else:
            self.count = 0;
            cv2.imshow('AR_TRACKER',frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('AR_TRACKER')
    ar_tracker = ARTracker()
    rospy.spin()