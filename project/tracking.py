import sys
import cv2
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

modelFile = "/home/ubuntu/armpi_fpv/src/object_tracking/scripts/models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
configFile = "/home/ubuntu/armpi_fpv/src/object_tracking/scripts/models/deploy.prototxt"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)
conf_threshold = 0.6

ik = ik_transform.ArmIK()

lock = RLock()

x_dis = 500
y_dis = 0.167
Z_DIS = 0.2
z_dis = Z_DIS
x_pid = PID.PID(P=0.06, I=0.005, D=0)
y_pid = PID.PID(P=0.00001, I=0, D=0)
z_pid = PID.PID(P=0.00003, I=0, D=0)


def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((0, y_dis, Z_DIS), -90, -92, -88)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1500, (
            (1, 200), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']),
            (6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)


def reset():
    global x_dis, y_dis, z_dis

    with lock:
        x_dis = 500
        y_dis = 0.167
        z_dis = Z_DIS

        x_pid.clear()
        y_pid.clear()
        z_pid.clear()


def init():
    global color_range

    rospy.loginfo("object tracking Init")
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server
    initMove()
    reset()


def run(img):
    global x_dis, y_dis, z_dis

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    area_max = 0
    center_x = 0
    center_y = 0

    blob = cv2.dnn.blobFromImage(img_copy, 0.5, (150, 150), [104, 117, 123], False, False)
    net.setInput(blob)
    detections = net.forward()
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            x1 = int(detections[0, 0, i, 3] * img_w)
            y1 = int(detections[0, 0, i, 4] * img_h)
            x2 = int(detections[0, 0, i, 5] * img_w)
            y2 = int(detections[0, 0, i, 6] * img_h)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, 8)
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            area_max = abs(x1 - x2)

    if abs(center_x - img_w / 2) < 100:
        x_pid.SetPoint = img_w / 2.0
        x_pid.update(center_x)
        dx = x_pid.output
        x_dis += int(dx)
        x_dis = 200 if x_dis < 200 else x_dis
        x_dis = 800 if x_dis > 800 else x_dis

        y_pid.SetPoint = 200

        # this part works as a sweet spot zone
        # if the face is in this zone, the arm does not move to adjust
        # currently commented

        # if abs(area_max - 200) < 20:
        #     area_max = 200

        y_pid.update(area_max)
        dy = y_pid.output
        y_dis += dy
        y_dis = 0.05 if y_dis < 0.05 else y_dis
        y_dis = 0.30 if y_dis > 0.30 else y_dis

        z_pid.SetPoint = img_h / 2.0
        z_pid.update(center_y)
        dy = z_pid.output
        z_dis += dy

        z_dis = 0.22 if z_dis > 0.22 else z_dis
        z_dis = 0.17 if z_dis < 0.17 else z_dis

        target = ik.setPitchRanges((0, round(y_dis, 4), round(z_dis, 4)), -90, -85, -95)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 20, (
                (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, x_dis)))
    return img


def image_callback(ros_image):
    global lock

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    frame = cv2_img.copy()
    frame_result = frame
    with lock:
        if __isRunning:
            frame_result = run(frame)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image

    image_pub.publish(ros_image)

