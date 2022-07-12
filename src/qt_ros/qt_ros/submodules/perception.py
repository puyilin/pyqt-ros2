from PyQt5 import QtWidgets
import numpy as np
import pyrealsense2 as rs
import cv2
from cmath import sqrt
#from .coordinate import get_aligned_images


class perception_node():
    def __init__(self):
        self.flag = 0
        self.open_camera()

    def open_camera(self):
        
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #这是打开相机
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
            self.align = rs.align(align_to)      # rs.align 执行深度帧与其他帧的对齐       
        except RuntimeError:
            QtWidgets.QMessageBox.warning(self, 'warning', "请检查相机于电脑是否连接正确", buttons=QtWidgets.QMessageBox.Ok)

    def image_show(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()            
        if  depth_frame or  color_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            self.color_image = np.asanyarray(color_frame.get_data())
            self.flag = 1
            return self.flag, self.color_image
        else:
            self.color_image = np.asanyarray(color_frame.get_data())
            self.flag =  0
            return self.flag, self.color_image

    def image_process(self):

        frame = self.color_image
        # 灰度处理
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 高斯滤波
        src = cv2.GaussianBlur(img_gray, (3, 3), 0)
        # 二值化处理
        ret, binary = cv2.threshold(src, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # canny边缘检测
        img_canny = cv2.Canny(binary, 80, 160)
        # 形态学操作
        k = np.ones((3, 3), dtype=np.uint8)
        img_binary = cv2.morphologyEx(img_canny, cv2.MORPH_CLOSE, k)        
        # 轮廓发现
        contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)              
        if contours is not None:  
            for i in range(len(contours)): 
                if len(contours[i]) > 50:
                    # 椭圆拟合
                    (cx, cy), (a, b), angle = cv2.fitEllipse(contours[i])
                    if (abs(a-b) < 5) and (200>a>60):
                        # 绘制椭圆和圆心
                        self.center_x = np.int(cx)
                        self.center_y = np.int(cy)
                        self.point0_x = self.center_x + np.int(a/2) 
                        self.point0_y = self.center_y 
                        cv2.ellipse(frame, (self.center_x, self.center_y),
                                    (np.int16(a/2), np.int16(b/2)), angle, 0, 360, (0, 255, 0), 2, 8, 0)
                        cv2.circle(frame, (self.center_x, self.center_y), 2, (255, 0, 0), -1, 2, 0)
                        cv2.circle(frame, (self.point0_x, self.point0_y), 2, (255, 0, 0), -1, 2, 0)
        return frame

    def get_aligned_images(self, pipeline, align):
    
        frames = pipeline.wait_for_frames()     # 等待获取图像帧，获取颜色和深度的框架集     
        aligned_frames = align.process(frames)      # 获取对齐帧，将深度框与颜色框对齐  

        aligned_depth_frame = aligned_frames.get_depth_frame()      # 获取对齐帧中的的depth帧 
        aligned_color_frame = aligned_frames.get_color_frame()      # 获取对齐帧中的的color帧

        #### 获取相机参数 ####
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics     # 获取深度参数（像素坐标系转相机坐标系会用到）
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics     # 获取相机内参


        #### 将images转为numpy arrays ####  
        img_color = np.asanyarray(aligned_color_frame.get_data())       # RGB图  
        img_depth = np.asanyarray(aligned_depth_frame.get_data())       # 深度图（默认16位）

        return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame


    def coordinate_get(self):
        colcor_intrin, depth_intrin, image_color, image_depth, aligned_depth_frame = self.get_aligned_images(self.pipeline, self.align)
        center_pixel = [self.center_x, self.center_y]
        point_pixel = [self.point0_x, self.point0_y]
        dis = aligned_depth_frame.get_distance(self.center_x, self.center_y)        # 获取该像素点对应的深度
        center_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, center_pixel, dis - 0.1)
        point_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, point_pixel, dis - 0.1)
        radius = sqrt((center_coordinate[0]-point_coordinate[0])**2 + (center_coordinate[1]-point_coordinate[1])**2)
        center_coordinate = str(center_coordinate)
        radius = str(radius)
        return center_coordinate, radius