
# controller_node.py
import rospy
from std_msgs.msg import String
import rtde_control
import rtde_receive
import cv2
import time
import math
import os
import json
import csv
import random
import A_main

# --------------------------
# 配置参数
# --------------------------
ROBOT_IP = "10.10.10.1"
HOME_WITH_TOOL = [-0.298569, -0.694446, 0.239335, 0.633457, -1.477861, 0.626266]
NEW_TCP = (0.0, 0.0, 0.26, 0.0, 0.0, 0.0)
TOTAL_CYCLES = 100
RUN_IDENTIFIER = "B"
PARENT_FOLDER = "material_4"
INITIAL_DESCENT = 0.01
HORIZONTAL_RANGE = 0.01

# 坐标变换（略）
def rotate_xy(x, y, theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return cos_t*x - sin_t*y, sin_t*x + cos_t*y

def transform_old_to_new(p): return [*rotate_xy(p[0],p[1],-math.radians(45)), p[2], p[3], p[4], p[5]]
def transform_new_to_old(p): return [*rotate_xy(p[0],p[1], math.radians(45)), p[2], p[3], p[4], p[5]]

class ControllerNode:
    def __init__(self):
        rospy.loginfo("Initializing RTDE...")
        self.rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        self.rtde_c.setTcp(NEW_TCP)
        time.sleep(0.5)
        self.rtde_c.moveL(HOME_WITH_TOOL, 0.2, 0.1)
        time.sleep(1)
        self.home_new = transform_old_to_new(HOME_WITH_TOOL)
        # Initialize camera & marker from A_main
        self.cam = A_main.init_camera(3)
        self.matcher = A_main.init_marker()
        os.makedirs(PARENT_FOLDER, exist_ok=True)

    def run(self):
        # Move to hover
        hover = self.home_new.copy(); hover[2] -= INITIAL_DESCENT
        self.rtde_c.moveL(transform_new_to_old(hover), 0.2, 0.1)
        time.sleep(1)
        for cycle in range(1, TOTAL_CYCLES+1):
            rospy.loginfo(f"Cycle {cycle}: position randomization and press-down")
            # random lateral shift
            shift = random.uniform(0, HORIZONTAL_RANGE)
            pos = hover.copy(); pos[1] += shift
            self.rtde_c.moveL(transform_new_to_old(pos), 0.2, 0.1); time.sleep(1)
            # random descent
            descent = random.uniform(INITIAL_DESCENT-0.0015, INITIAL_DESCENT+0.0015)
            bottom = pos.copy(); bottom[2] -= descent
            self.rtde_c.moveL(transform_new_to_old(bottom), 0.2, 0.1); time.sleep(0.5)
            # capture frame & flow via A_main
            frame, flow = A_main.capture_frame_and_flow(self.cam, self.matcher)
            # save outputs
            folder = os.path.join(PARENT_FOLDER, f"cycle_{cycle}_{RUN_IDENTIFIER}")
            os.makedirs(folder, exist_ok=True)
            # image
            cv2.imwrite(os.path.join(folder, "frame.png"), frame)
            # flow JSON
            with open(os.path.join(folder, "markerflow.json"), 'w') as jf:
                json.dump({'cycle': cycle, 'flow': flow}, jf, indent=2)
            # flow CSV
            with open(os.path.join(folder, "markerflow.csv"), 'w', newline='') as cf:
                writer = csv.writer(cf)
                writer.writerow(['Ox','Oy','Cx','Cy','Occupied'])
                # assume flat lists
                writer.writerow([flow['Ox'], flow['Oy'], flow['Cx'], flow['Cy'], flow['Occupied']])
            # back to hover
            self.rtde_c.moveL(transform_new_to_old(pos), 0.2, 0.1); time.sleep(0.5)
        # finish
        self.rtde_c.moveL(HOME_WITH_TOOL, 0.2, 0.1)
        time.sleep(1)
        self.rtde_c.servoStop(); self.rtde_c.stopScript()

if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=True)
    ControllerNode().run()


# import rospy
# from std_msgs.msg import String
# import cv2
# import numpy as np
# import time
# import math
# import sys
# import os
# import json
# import random

# # RTDE 接口
# try:
#     import rtde_control
#     import rtde_receive
# except ImportError:
#     rospy.logerr("请先安装 rtde_control, rtde_receive 等库")
#     sys.exit(1)

# import find_marker
# import A_utility
# import setting

# # --------------------------
# # 配置参数
# # --------------------------
# ROBOT_IP = "10.10.10.1"
# HOME_WITH_TOOL = [-0.298569, -0.694446, 0.239335, 0.633457, -1.477861, 0.626266]
# NEW_TCP = (0.0, 0.0, 0.26, 0.0, 0.0, 0.0)
# DT = 0.01
# LOOKAHEAD_TIME = 0.1
# GAIN = 300

# # 运行参数
# TOTAL_CYCLES = 100
# RUN_IDENTIFIER = "B"
# PARENT_FOLDER = "material_4"
# INITIAL_DESCENT = 0.01
# HORIZONTAL_RANGE = 0.01  # 随机水平位置范围 [0, 1cm]

# # --------------------------
# # 坐标转换与插值函数
# # --------------------------
# def rotate_xy(x, y, theta):
#     cos_t = math.cos(theta)
#     sin_t = math.sin(theta)
#     return cos_t * x - sin_t * y, sin_t * x + cos_t * y

# def transform_old_to_new(pose_old):
#     x_old, y_old, z, rx, ry, rz = pose_old
#     x_new, y_new = rotate_xy(x_old, y_old, -math.radians(45))
#     return [x_new, y_new, z, rx, ry, rz]

# def transform_new_to_old(pose_new):
#     x_new, y_new, z, rx, ry, rz = pose_new
#     x_old, y_old = rotate_xy(x_new, y_new, math.radians(45))
#     return [x_old, y_old, z, rx, ry, rz]

# # --------------------------
# # ControllerNode：集成新传感器，仅下压采集
# # --------------------------
# class ControllerNode:
#     def __init__(self):
#         # 初始化 RTDE
#         try:
#             self.rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
#             self.rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
#         except Exception as e:
#             rospy.logerr(f"连接RTDE失败: {e}")
#             sys.exit(1)
#         self.rtde_c.setTcp(NEW_TCP)
#         time.sleep(0.3)
#         self.rtde_c.moveL(HOME_WITH_TOOL, 0.2, 0.1)
#         time.sleep(1)
#         self.home_pose_new = transform_old_to_new(HOME_WITH_TOOL)

#         # 初始化相机与标记追踪
#         self.cam = cv2.VideoCapture(0)
#         self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
#         self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
#         setting.init()
#         self.m = find_marker.Matching(
#             N_=setting.N_, M_=setting.M_, fps_=setting.fps_,
#             x0_=setting.x0_, y0_=setting.y0_, dx_=setting.dx_, dy_=setting.dy_
#         )

#     def capture_sensor_data(self):
#         # 获取处理过的图像
#         frame = A_utility.get_processed_frame(self.cam)
#         # 计算 markerflow
#         m_centers = A_utility.marker_center(frame)
#         self.m.init(m_centers)
#         self.m.run()
#         flow = self.m.get_flow()
#         # 转换为列表格式以便序列化
#         Ox, Oy, Cx, Cy, Occ = flow
#         flow_data = {
#             'Ox': list(Ox),
#             'Oy': list(Oy),
#             'Cx': list(Cx),
#             'Cy': list(Cy),
#             'Occupied': list(Occ)
#         }
#         return frame, flow_data

#     def run(self):
#         rospy.loginfo("从 HOME_WITH_TOOL 下降1cm 到悬停位...")
#         hover = list(self.home_pose_new)
#         hover[2] -= INITIAL_DESCENT
#         self.rtde_c.moveL(transform_new_to_old(hover), 0.2, 0.1)
#         time.sleep(2)

#         # 确保父目录
#         os.makedirs(PARENT_FOLDER, exist_ok=True)

#         for cycle in range(1, TOTAL_CYCLES+1):
#             rospy.loginfo(f"周期 {cycle}：随机水平位置调整与下压采集")
#             # 随机水平偏移
#             shift = random.uniform(0, HORIZONTAL_RANGE)
#             pos = list(hover)
#             pos[1] += shift
#             self.rtde_c.moveL(transform_new_to_old(pos), 0.2, 0.1)
#             time.sleep(1)
#             # 随机下降深度
#             descent = random.uniform(INITIAL_DESCENT-0.0015, INITIAL_DESCENT+0.0015)
#             bottom = list(pos)
#             bottom[2] -= descent
#             self.rtde_c.moveL(transform_new_to_old(bottom), 0.2, 0.1)
#             time.sleep(0.5)

#             # 采集图像与 markerflow
#             frame, flow_data = self.capture_sensor_data()

#             # 保存数据
#             folder = os.path.join(PARENT_FOLDER, f"cycle_{cycle}_{RUN_IDENTIFIER}")
#             os.makedirs(folder, exist_ok=True)
#             cv2.imwrite(os.path.join(folder, "frame.png"), frame)
#             with open(os.path.join(folder, "markerflow.json"), 'w') as jf:
#                 json.dump({'cycle': cycle, 'flow': flow_data}, jf, indent=2)

#             # 上升回悬停位
#             self.rtde_c.moveL(transform_new_to_old(pos), 0.2, 0.1)
#             time.sleep(1)

#         rospy.loginfo("所有周期采集完成，返回 HOME_WITH_TOOL...")
#         self.rtde_c.moveL(HOME_WITH_TOOL, 0.2, 0.1)
#         time.sleep(1)
#         self.rtde_c.servoStop()
#         self.rtde_c.stopScript()


# def main():
#     rospy.init_node("controller_node", anonymous=True)
#     node = ControllerNode()
#     node.run()

# if __name__ == "__main__":
#     main()

