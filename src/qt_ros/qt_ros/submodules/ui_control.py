import sys
from cmath import sqrt
import cv2
from PyQt5.QtWidgets import QApplication,QMainWindow, QGridLayout
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import QBasicTimer
# 引入窗口控件类
from .Ui_mainwin import Ui_MainWindow
# 在GUI中显示matplotlib图形
import matplotlib
matplotlib.use("Qt5Agg")  # 声明使用QT5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
# 引入计算库
import numpy as np
# 引入环境吸引域
from .ARIE import ARIE_formulation
from .coordinate import get_aligned_images
# 插入背景图片
from .image_rc import qt_resource_data
# 引入ros相关
import rclpy
import rclpy
import threading
from rclpy import executors
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from PyQt5 import QtCore, QtGui, QtWidgets
import pyrealsense2 as rs

class ControlApp(QMainWindow):
    def __init__(self, mainwindow):
        super(ControlApp, self).__init__()
        self.Ui_main = Ui_MainWindow() 
        self.ControlGui(mainwindow)
        

    def ControlGui(self, mainwindow):  
        self.Ui_main.setupUi(mainwindow)
        ####选择任务类型&&&   
        #self.radioButton.setChecked(True)
        self.Ui_main.AssembleButton.toggled['bool'].connect(self.buttonState)
        self.Ui_main.CaptureButton.toggled['bool'].connect(self.buttonState)
        self.Ui_main.OtherTaskButton.toggled['bool'].connect(self.buttonState) 
        ####选择任务类型&&&

        ####选择零件类型
        self.Ui_main.PartType.currentIndexChanged.connect(self.selectionChange) 
        ####选择零件类型

        ####输入半径、精度、速度、接触力大小
        self.Ui_main.radius.setValidator(QDoubleValidator(0.00,99.99,3))
        self.Ui_main.radius.returnPressed.connect(self.textInput_radius)        
        self.Ui_main.velocity.setValidator(QDoubleValidator(0.00,99.99,3))
        self.Ui_main.velocity.returnPressed.connect(self.textInput_velocity)
        self.Ui_main.precision.setValidator(QDoubleValidator(0.00,99.99,3))
        self.Ui_main.precision.returnPressed.connect(self.textInput_precision)
        self.Ui_main.contactForce.setValidator(QDoubleValidator(0.00,99.99,3))
        self.Ui_main.contactForce.returnPressed.connect(self.textInput_contactForce)
        ####输入半径、精度、速度、接触力大小

        ###选择相机、力觉传感器、机器人本体 
        #选相机     
        self.Ui_main.realsense.clicked.connect(self.cameraButtonClick)
        self.Ui_main.kinect.clicked.connect(self.cameraButtonClick)
        self.Ui_main.mercury.clicked.connect(self.cameraButtonClick)
        #选传感器
        self.Ui_main.axia.clicked.connect(self.sensorButtonClick)
        self.Ui_main.onrobot.clicked.connect(self.sensorButtonClick)
        self.Ui_main.lireach.clicked.connect(self.sensorButtonClick)
        #选机器人本体
        self.Ui_main.kinova.clicked.connect(self.robotButtonClick)
        self.Ui_main.rokae.clicked.connect(self.robotButtonClick)
        self.Ui_main.kuka.clicked.connect(self.robotButtonClick)
        ###选择相机、力传感器、机器人本体

        ###参数显示        
        self.Ui_main.ParameterEstimate.clicked.connect(self.parameterview)
        ###参数显示

        ###原图与处理图显示
        self.timer_camera = QtCore.QTimer()  # 定义定时器，用于控制显示视频的帧率
        self.cap = cv2.VideoCapture()  # 视频流
        self.CAM_NUM = 8  # 为0时表示视频流来自笔记本内置摄像头
        self.timer_camera.timeout.connect(self.show_camera)  # 若定时器结束，则调用show_camera()
        self.Ui_main.RawImage.clicked.connect(self.button_open_camera_clicked)
        self.timer_camera.timeout.connect(self.show_processed_image)
        self.Ui_main.ProcessingImage.clicked.connect(self.chang_button_text)
        ###原图显示

        ####输入初始偏转角大小
        # self.Ui_main.thetaX.setValidator(QIntValidator(0,90)) # 限制角度输入为0~90°
        self.Ui_main.thetaX.returnPressed.connect(self.angleInput_x)    
        # self.Ui_main.thetaY.setValidator(QIntValidator(0,90)) # 限制角度输入为0~90°
        self.Ui_main.thetaY.returnPressed.connect(self.angleInput_y)
        # self.Ui_main.thetaZ.setValidator(QIntValidator(0,90)) # 限制角度输入为0~90°
        self.Ui_main.thetaZ.returnPressed.connect(self.angleInput_z)


        ###实现pushButton功能
        # 构建环境吸引域
        self.Ui_main.AttactiveRegion.clicked.connect(self.ARIE_construct)
        # 选择方案
        self.Ui_main.CommonTwostep.clicked.connect(self.strategyChoose)
        self.Ui_main.InclineTwostep.clicked.connect(self.strategyChoose)
        self.Ui_main.OnlineStrategy.clicked.connect(self.strategyChoose)
        # 选择控制方法
        self.Ui_main.PositionCotrol.clicked.connect(self.methodChoose)
        self.Ui_main.ImpedanceControl.clicked.connect(self.methodChoose)
        self.Ui_main.ForceControl.clicked.connect(self.methodChoose)
        # 传感器数据图绘制的开始、暂停、终止按钮
        self.Ui_main.StartButton.clicked.connect(self.startTimer)
        self.Ui_main.SuspendButton.clicked.connect(self.suspendTimer)
        self.Ui_main.EndButton.clicked.connect(self.endTimer)
        # 进度条的开始、暂停、终止按钮
        self.Ui_main.StartButton.clicked.connect(self.myTimerStart)
        self.Ui_main.SuspendButton.clicked.connect(self.myTimerSuspend)
        self.Ui_main.EndButton.clicked.connect(self.myTimerEnd)
        ###实现pushButton功能

        ###进度条显示
        # 配置一个值表示进度条的当前进度
        self.pv = 0 
        # 申明一个时钟控件
        self.timer1 = QBasicTimer()
        # 设置进度条的范围
        self.Ui_main.progressBar.setMinimum(0)
        self.Ui_main.progressBar.setMaximum(100)
        self.Ui_main.progressBar.setValue(self.pv)
        ###进度条显示

        ###传感器数据绘图
        self.drawTime = QtCore.QTimer(self)
        self.drawTime.timeout.connect(self.showTime)
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.gridlayout = QGridLayout(self.Ui_main.TipPosition)  # 继承容器groupBox       
        self.gridlayout.addWidget(self.canvas,0,1)
        self.x=[]
        ###传感器数据绘图

        ###各类节点初始化
        self.perception_node_init()


    '''
        任务类型选择槽函数
    '''
    def buttonState(self):
        radioButton = self.sender()
        if radioButton.isChecked() == True:
            # print('<' + radioButton.text() + '> 被选中')
            self.Ui_main.OrdersView.append('您选择的任务类型为:' + radioButton.text())   #文本框逐条添加数据
            self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    '''
        零件类型选择槽函数
    '''
    def selectionChange(self, i):
        # print('current index',i,'selection changed', self.Ui_main.PartType.currentText())
        self.Ui_main.OrdersView.append('您选择的零件类型为:' + self.Ui_main.PartType.currentText())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    '''
        半径、精度、速度、接触力要求输入槽函数
    '''   
    def textInput_radius(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('您输入的孔半径为:' + str.text() + ' mm')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        radius = float(str.text())
        self.radius = round(radius)

    def textInput_velocity(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('您输入的速度要求为：小于' + str.text() + 'm/s')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    def textInput_precision(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('您输入的精度要求为:' + str.text() + ' mm')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        
    def textInput_contactForce(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('您输入的接触力要求为：小于' + str.text() + ' N')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    def textInput(self):
        str = self.sender()
    #    str1 = self.Ui_main.radius.text()
        print('你输入的数字是:' + str.text())

    '''
        角度输入槽函数
    '''   
    def angleInput_x(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('初始状态绕x轴偏转:' + str.text() + '°')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        # print('绕x轴偏转:' + str.text() + '°')
        self.angle_x = int(str.text())
    
    def angleInput_y(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('初始状态绕y轴偏转:' + str.text() + '°')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        # print('绕y轴偏转:' + str.text() + '°')
        self.angle_y = int(str.text())    

    def angleInput_z(self):
        str = self.sender()
        self.Ui_main.OrdersView.append('初始状态绕z轴偏转:' + str.text() + '°')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        # print('绕z轴偏转:' + str.text() + '°')
        self.angle_z = int(str.text())

    """
        构建环境吸引域函数 
    """
    def ARIE_construct(self):

        self.Ui_main.OrdersView.append('环境吸引域构建开始!')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

        # time.sleep(1)
        r = self.radius
        theta_x = self.angle_x; theta_y = self.angle_y; theta_z = self.angle_z

        self.arie = ARIE_formulation(r,theta_x,theta_y,theta_z) # ARIE构建对象，输入半径和三个偏转角
        self.arie.signal.connect(self.displayProgress)   
        self.arie.start() # 开启子进程

        X,Y,lowest_z = self.arie.ARIE_calculation() # 调用计算函数

        # 画图
        draw_ARIE = Figure_Canvas()
        draw_ARIE.draw_fig(X,Y,lowest_z)
        graphicsence = QtWidgets.QGraphicsScene() # 先声明一个场景控件
        graphicsence.addWidget(draw_ARIE) # 将图定义为一个场景控件
        self.Ui_main.ARIE_view.setScene(graphicsence) # 将QGraphicView和场景链接起来
        self.Ui_main.ARIE_view.show()

        # 交互命令行显示
        self.Ui_main.OrdersView.append('环境吸引域构建完成!')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        self.arie.stop()
        
        # self.Ui_main.ARIE_view.fitInView(graphicsence.sceneRect())

    '''
        相机、传感器、机器人比如难题选择的槽函数 
    '''
    def cameraButtonClick(self):
        Camera = self.sender()
        # print('你按下的按钮是:' + Camera.text())
        self.Ui_main.OrdersView.append('您选择的视觉传感为:' + Camera.text())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            #config.enable_device_from_file("666.bag")#这是打开相机录制的视频
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #这是打开相机
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
            self.align = rs.align(align_to)      # rs.align 执行深度帧与其他帧的对齐
        except RuntimeError:
            QtWidgets.QMessageBox.warning(self, 'warning', "请检查相机于电脑是否连接正确", buttons=QtWidgets.QMessageBox.Ok)


    def sensorButtonClick(self):
        sensor = self.sender()
        # print('你按下的按钮是:' + sensor.text())
        self.Ui_main.OrdersView.append('您选择的力觉传感为:' + sensor.text())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    def robotButtonClick(self):
        robot = self.sender()
        # print('你按下的按钮是:' + robot.text())
        self.Ui_main.OrdersView.append('您选择的机器人型号为:' + robot.text())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部


    '''
        方案选择
    '''
    def strategyChoose(self):
        strategy = self.sender()
        # print("你按下的按钮是" + strategy.text())
        self.Ui_main.OrdersView.append('您选择的策略为:' + strategy.text())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    '''
        控制方法选择
    '''
    def methodChoose(self):
        method = self.sender()
        # print("你按下的按钮是" + method.text())
        self.Ui_main.OrdersView.append('您选择的控制方法为:' + method.text())   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    '''
        显示视频的槽函数
    '''
    def button_open_camera_clicked(self):
        if self.timer_camera.isActive() == False:  # 若定时器未启动
            self.timer_camera.start(30)  # 定时器开始计时30ms，结果是每过30ms从摄像头中取一帧显示
            self.Ui_main.RawImage.setText('关闭相机')
        else:
            self.timer_camera.stop()  # 关闭定时器
            cv2.destroyAllWindows()
            self.Ui_main.rawImageShow.clear()  # 清空视频显示区域
            self.Ui_main.RawImage.setText('原图')
    def show_camera(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()            
        if  depth_frame or  color_frame:
        # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            self.color_image = np.asanyarray(color_frame.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            image = cv2.resize(self.color_image, (160,120))
            show = cv2.resize(image, (160, 120))  # 把读到的帧的大小重新设置为 640x480
            #show = cv2.cvtColor(show, cv2.COLOR_BGR2RGB)  # 视频色彩转换回RGB，这样才是现实的颜色
            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                    QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
            self.Ui_main.rawImageShow.setPixmap(QtGui.QPixmap.fromImage(showImage))  # 往显示视频的Label里 显示QImage

    def chang_button_text(self):
        if self.Ui_main.ProcessingImage.text() == "处理图":
            self.Ui_main.ProcessingImage.setText("关闭图片")
        else:
            self.Ui_main.ProcessingImage.setText("处理图")
        

    def show_processed_image(self):

        if self.Ui_main.ProcessingImage.text() == "关闭图片":
            
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
            #draw0 = cv2.drawContours(frame, contours, -1, (255, 0, 0), 3)
            #cv2.circle(frame, (100, 100), 2, (255, 0, 0), -1, 2, 0)
            #cv2.circle(frame, (400, 100), 2, (255, 0, 0), -1, 2, 0)            
            '''
            if contours is not None:
                for index in range(len(contours)):
                    (x, y), radius = cv2.minEnclosingCircle(contours[index])
                    if 150> radius > 50:
                        center = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(frame, center, radius, (0, 255, 0), 2)
            '''           
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
            # 显示处理的图片
            show_1 = cv2.resize(frame, (160, 120))  # 把读到的帧的大小重新设置为 640x480
            show_1 = cv2.cvtColor(show_1, cv2.COLOR_BGR2RGB)  # 视频色彩转换回RGB，这样才是现实的颜色
            showImage_1 = QtGui.QImage(show_1.data, show_1.shape[1], show_1.shape[0],
                                 QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
            self.Ui_main.processedShow.setPixmap(QtGui.QPixmap.fromImage(showImage_1))
        else:
            self.Ui_main.processedShow.setPixmap(QtGui.QPixmap(""))    
 
    '''
        参数显示槽函数
    '''
    def parameterview(self):

        if self.timer_camera.isActive() == True:          
            colcor_intrin, depth_intrin, image_color, image_depth, aligned_depth_frame = get_aligned_images(self.pipeline, self.align)
            center_pixel = [self.center_x, self.center_y]
            point_pixel = [self.point0_x, self.point0_y]
            dis = aligned_depth_frame.get_distance(self.center_x, self.center_y)        # 获取该像素点对应的深度
            center_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, center_pixel, dis - 0.1)
            point_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, point_pixel, dis - 0.1)
            radius = sqrt((center_coordinate[0]-point_coordinate[0])**2 + (center_coordinate[1]-point_coordinate[1])**2)
            center_coordinate = str(center_coordinate)
            radius = str(radius)
            if self.Ui_main.ParameterEstimate.text() == "参数估计":
                if self.Ui_main.PartType.currentText() == "圆轴圆孔":
                    self.Ui_main.ParameterView_1.setText("中心位置:"+center_coordinate)
                    self.Ui_main.ParameterView_2.setText("半径:"+radius)
                else:
                    self.Ui_main.ParameterView_1.setText("中心位置")
                    self.Ui_main.ParameterView_2.setText("边数")
                self.Ui_main.ParameterEstimate.setText("关闭")
            else:
                self.Ui_main.ParameterView_1.setText("")
                self.Ui_main.ParameterView_2.setText("")
                self.Ui_main.ParameterEstimate.setText("参数估计")


    '''
        进度条槽函数
    '''
    def myTimerStart(self):
        if self.timer1.isActive() == False:
            self.Ui_main.OrdersView.append('开始执行任务')   #文本框逐条添加数据
            self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
            self.timer1.start(100, self)


    def myTimerSuspend(self):
        if self.timer1.isActive():
            self.Ui_main.OrdersView.append('暂停执行任务')   #文本框逐条添加数据
            self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部
            self.timer1.stop()


    def myTimerEnd(self):
        self.timer1.stop()
        self.pv = 0
        self.Ui_main.progressBar.setValue(self.pv)
        self.Ui_main.OrdersView.append('终止执行任务')   #文本框逐条添加数据
        self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End)  #文本框显示到底部

    def timerEvent(self, e):
        if self.pv == 100:
            # 进度条停止
            self.timer1.stop()
            # 画图停止
            self.drawTime.stop() # 计时停止
            self.x=[] # 清空数组
            self.Ui_main.OrdersView.append('任务完成') # 文本框逐条添加数据
            self.Ui_main.OrdersView.moveCursor(self.Ui_main.OrdersView.textCursor().End) # 文本框显示到底部
        else:
            self.pv += 1
            self.Ui_main.progressBar.setValue(self.pv)
    
    # 显示实时进度：构建ARIE子进程的对接函数
    def displayProgress(self,mes):
        self.Ui_main.progressBar.setValue(mes)

    '''
        绘图槽函数
    '''
    def showTime(self):
        randomData = np.random.random_sample()*10 # 返回一个[0,1)之间的浮点型随机数*10
        self.x.append(randomData) # 数组更新  
        self.ax.clear()
        self.ax.plot(self.x)
        self.canvas.draw()

    def startTimer(self):
        # 设置计时间隔并启动
        self.ax = self.figure.add_axes([0.1, 0.1, 0.8, 0.8]) # 创建图窗
        self.drawTime.start(500) # 每隔一秒执行一次绘图函数 showTime

    def suspendTimer(self):
        # 设置计时间隔并启动
        self.drawTime.stop() # 每隔一秒执行一次绘图函数 showTime

    def endTimer(self):
        self.drawTime.stop() # 计时停止
        self.x=[] # 清空数组

    '''
        命令行显示槽函数
    '''
    def ordersView(self):
        order = self.sender()
        self.Ui_main.OrdersView.setText("       " + order.text())

    '''
        节点初始化
    '''
    def perception_node_init(self):
        self.perception_node = rclpy.create_node('perception_node')
        self.coordinate_pub = self.perception_node.create_publisher(String, '/coordinate_radius', 
            qos_profile=qos_profile_sensor_data)
        self.force_figure_pub = self.perception_node.create_publisher(Float32MultiArray, '/force_fihure', 
            qos_profile=qos_profile_sensor_data)

    def destroy_nodes(self):
        self.perception_node.destroy_node()
        

""" 
    构建环境吸引域创建的画图类 
"""

class Figure_Canvas(FigureCanvas):

    def __init__(self, parent = None, width = 3, height = 2):
        fig = Figure(figsize=(width,height), dpi=96)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        self.ax = fig.gca(projection='3d')

    def draw_fig(self,X,Y,lowest_z):

        self.ax.plot_surface(X,Y,lowest_z,cmap='rainbow')
        self.ax.set_xlabel('x', fontsize = 10)
        self.ax.set_ylabel('y', fontsize = 10)
        self.ax.set_zlabel('z', fontsize = 10)
        self.ax.xaxis.set_tick_params(labelsize=6)
        self.ax.yaxis.set_tick_params(labelsize=6)
        self.ax.zaxis.set_tick_params(labelsize=6)

if  __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    # 控制类的合体
    Control = ControlApp(mainWindow)
    # 显示窗口 
    mainWindow.show()
    sys.exit(app.exec_())
