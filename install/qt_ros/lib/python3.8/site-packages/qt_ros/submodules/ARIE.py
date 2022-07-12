import numpy as np
import math
import time
import sys
from PyQt5.QtCore import *


""" 创建ARIE构建类为一个子线程"""

class ARIE_formulation(QThread):

    signal = pyqtSignal(int) # 通过类成员对象定义信号对象

    def __init__(self,r,theta_x,theta_y,theta_z):

        super(ARIE_formulation, self).__init__()

        ''' 初始化参数设置 '''
        # 零件尺寸
        self.R=r
        self.H=self.R*2/3

        # 分辨率
        self.resolution = np.array([60,60,60])

        # 设置n边形顶点坐标 'vertex_pi' refers to 'vertex_position_initial' 'vertex_pc' : vertex_position_current
        self.vertex_num = 60
        self.vertex_pi = np.zeros([4,self.vertex_num*2])
        theta = np.linspace(0,2*math.pi,num=self.vertex_num)
        self.vertex_pi[0,0:self.vertex_num] = self.R*np.cos(theta)
        self.vertex_pi[1,0:self.vertex_num] = self.R*np.sin(theta)
        self.vertex_pi[2,0:self.vertex_num] = 0
        self.vertex_pi[3,0:self.vertex_num] = 1
        self.vertex_pi[:,self.vertex_num:self.vertex_num*2] = self.vertex_pi[:,0:self.vertex_num]
        self.vertex_pc = np.zeros([4,self.vertex_num*2])

        # 初始转角设置 alpha:rotation around x-axis
        self.alpha = theta_x/180*math.pi
        self.beta = theta_y/180*math.pi
        self.gamma = theta_z/180*math.pi
        self.schedule  = 0.01 # 记录进程

        # print(vertex_position_initial[1,:])

    
    def ARIE_calculation(self):

        '''数据计算'''

        # 旋转矩阵 rotation matrices
        rotate_x = np.array([
            [1,0,0,0],
            [0,math.cos(self.alpha),-math.sin(self.alpha),0],
            [0,math.sin(self.alpha),math.cos(self.alpha),0],
            [0,0,0,1]])
        rotate_y = np.array([
            [math.cos(self.beta),0,math.sin(self.beta),0],
            [0,1,0,0],
            [-math.sin(self.beta),0,math.cos(self.beta),0],
            [0,0,0,1]])
        rotate_z = np.array([
            [math.cos(self.gamma),-math.sin(self.gamma),0,0],
            [math.sin(self.gamma),math.cos(self.gamma),0,0],
            [0,0,1,0],
            [0,0,0,1]])

        # Tt = (rotate_x * rotate_y) * rotate_z '*' 是点乘!
        Tt = np.dot(np.dot(rotate_x,rotate_y),rotate_z)
        Tt[0:3,3] = np.array([0,0,0]).T

        self.vertex_pc[:,0:self.vertex_num] = np.dot(Tt,self.vertex_pi[:,0:self.vertex_num])
        self.vertex_pc[:,self.vertex_num:self.vertex_num*2] = self.vertex_pc[:,0:self.vertex_num]

        # print(Tt)
        # print(vertex_position_current[2,0:vertex_num])

        # 方向向量
        vector_dir = np.dot(Tt,np.array([0,0,self.H,0]).T)

        min_init_x = np.min(self.vertex_pi[0,0:self.vertex_num])
        max_init_x = np.max(self.vertex_pi[0,0:self.vertex_num])
        min_init_y = np.min(self.vertex_pi[1,0:self.vertex_num])
        max_init_y = np.max(self.vertex_pi[1,0:self.vertex_num])


        # 旋转标志 1(+) & 2(-) for rotate x-axis, 3(+) & 4(-) for ratate y-axis
        flag_rotation = 0

        # 平移因子
        scale_buff = 0.5

        temp_b = np.min(self.vertex_pc[2,0:self.vertex_num])

        if self.beta == 0 and self.alpha > 0: # 绕x轴正转
            temp_a = np.min(self.vertex_pc[1,0:self.vertex_num])# temp_a<0
            x = np.linspace(min_init_x-scale_buff,max_init_x+scale_buff,num=self.resolution[0])
            y = np.linspace(min_init_y-temp_a-scale_buff,max_init_y-temp_a+scale_buff,num=self.resolution[1])
            z = np.linspace(temp_a*math.tan(self.alpha),-temp_b,num=self.resolution[2])
            flag_rotation = 1

        elif self.beta == 0 and self.alpha < 0: # 绕x轴反转
            temp_a = np.max(self.vertex_pc[1,0:self.vertex_num])# temp_a>0
            x = np.linspace(min_init_x-scale_buff,max_init_x+scale_buff,num=self.resolution[0])
            y = np.linspace(min_init_y-temp_a-scale_buff,max_init_y-temp_a+scale_buff,num=self.resolution[1])
            z = np.linspace(temp_a*math.tan(self.alpha),-temp_b,num=self.resolution[2])
            flag_rotation = 2  

        elif self.alpha == 0 and self.beta > 0: # 绕y轴正转
            temp_a = np.max(self.vertex_pc[0,0:self.vertex_num])# temp_a>0
            x = np.linspace(min_init_x-temp_a-scale_buff,max_init_x-temp_a+scale_buff,num=self.resolution[0])
            y = np.linspace(min_init_y-scale_buff,max_init_y+scale_buff,num=self.resolution[1])
            z = np.linspace(-temp_a*math.tan(self.beta),-temp_b,num=self.resolution[2])
            flag_rotation = 3  

        elif self.alpha == 0 and self.beta < 0: # 绕y轴反转
            temp_a = np.min(self.vertex_pc[0,0:self.vertex_num])# temp_a>0
            x = np.linspace(min_init_x-temp_a-scale_buff,max_init_x-temp_a+scale_buff,num=self.resolution[0])
            y = np.linspace(min_init_y-scale_buff,max_init_y+scale_buff,num=self.resolution[1])
            z = np.linspace(-temp_a*math.tan(self.beta),-temp_b,num=self.resolution[2])
            flag_rotation = 4

        else:
            print('Please rotate around one axis!')  

        length_x = len(x); length_y = len(y); length_z = len(z)
        lowest_z = np.zeros([length_x,length_y])

        # print(z)

        """ 开始计算 """

        for i in range(length_x):
            for j in range(length_y):

                self.schedule = round(i*100/length_x,2)

                # print(self.schedule)
                cross_line_num = 0 # 与穿过直线交点的个数

                if flag_rotation == 1:
                    for m in range(self.vertex_num):
                        if self.vertex_pi[1,m] - temp_a > y[j] and \
                        ((self.vertex_pi[0,m] < x[i] and self.vertex_pi[0,m+1] > x[i]) or \
                        (self.vertex_pi[0,m] > x[i] and self.vertex_pi[0,m+1] < x[i])):
                            cross_line_num += 1
                elif flag_rotation == 2:
                    for m in range(self.vertex_num):
                        if self.vertex_pi[1,m] - temp_a > y[j] and \
                            ((self.vertex_pi[0,m] < x[i] and self.vertex_pi[0,m+1] > x[i]) or \
                            (self.vertex_pi[0,m] > x[i] and self.vertex_pi[0,m+1] < x[i])):
                            cross_line_num += 1
                elif flag_rotation == 3:
                    for m in range(self.vertex_num):
                        if self.vertex_pi[1,m] > y[j] and \
                            ((self.vertex_pi[0,m] - temp_a < x[i] and self.vertex_pi[0,m+1] - temp_a > x[i]) or \
                            (self.vertex_pi[0,m] - temp_a > x[i] and self.vertex_pi[0,m+1] - temp_a < x[i])):
                            cross_line_num += 1
                elif flag_rotation == 4:
                    for m in range(self.vertex_num):
                        if self.vertex_pi[1,m] > y[j] and \
                            ((self.vertex_pi[0,m] - temp_a < x[i] and self.vertex_pi[0,m+1] - temp_a > x[i]) or \
                            (self.vertex_pi[0,m] - temp_a > x[i] and self.vertex_pi[0,m+1] - temp_a < x[i])):
                            cross_line_num += 1
                
                for k in range(length_z):

                    if cross_line_num % 2 == 0:
                        lowest_z[i,j] = -z[k]
                        break
                
                    Tt[0:3,3] = np.array([x[i],y[j],-z[k]]).T

                    self.vertex_pc[:,0:self.vertex_num] = np.dot(Tt,self.vertex_pi[:,0:self.vertex_num])
                    self.vertex_pc[:,self.vertex_num:self.vertex_num*2] = self.vertex_pc[:,0:self.vertex_num]
                    vector_dir = np.dot(Tt,np.array([0,0,self.H,0]).T) # 方向向量

                    # 计算切面
                    section_num = 0
                    section_position = np.zeros([4,self.vertex_num+1])
                    section_position[3,:] = 1
                    under_flag = 0 ; under_index = 0

                    for temp_i in range(self.vertex_num):
                        if self.vertex_pc[2,temp_i] < 0:
                            under_flag = 1
                            under_index = temp_i
                            break

                    if under_flag == 1:

                        for temp_i in range(self.vertex_num):
                            temp_j = temp_i + under_index
                            if self.vertex_pc[2,temp_j] <= 0:
                                section_position[0:2,section_num] = self.vertex_pc[0:2,temp_j] - \
                                    (self.vertex_pc[2,temp_j]/vector_dir[2])*vector_dir[0:2]
                                section_num += 1    
                            else:
                                if self.vertex_pc[2,temp_j-1] < 0:
                                    section_position[0:2,section_num] = self.vertex_pc[0:2,temp_j-1] + \
                                        (-self.vertex_pc[2,temp_j-1]/(self.vertex_pc[2,temp_j]-self.vertex_pc[2,temp_j-1])) * \
                                        (self.vertex_pc[0:2,temp_j] - self.vertex_pc[0:2,temp_j-1])
                                    section_num += 1
                                elif self.vertex_pc[2,temp_j+1] < 0:
                                    section_position[0:2,section_num] = self.vertex_pc[0:2,temp_j] + \
                                        (-self.vertex_pc[2,temp_j]/(self.vertex_pc[2,temp_j+1]-self.vertex_pc[2,temp_j])) * \
                                        (self.vertex_pc[0:2,temp_j+1] - self.vertex_pc[0:2,temp_j])
                                    section_num += 1  

                    # 判断是否接触
                    contact_flag = 0; flag_out = 1; flag_in = 1; temp = 0; cross_num = 0   

                    if under_flag == 1:

                        for temp_i in range(section_num):
                            for temp_j in range(self.vertex_num):
                                if self.vertex_pi[1,temp_j] >= section_position[1,temp_i] and \
                                    ((self.vertex_pi[0,temp_j] < section_position[0,temp_i] and self.vertex_pi[0,temp_j+1] > section_position[0,temp_i]) or \
                                    (self.vertex_pi[0,temp_j] > section_position[0,temp_i] and self.vertex_pi[0,temp_j+1] < section_position[0,temp_i]) or \
                                    (self.vertex_pi[0,temp_j] == section_position[0,temp_i])):
                                    cross_num += 1
                            
                            if cross_num % 2 == 1: # 奇数个交点
                                flag_in = 0 # 点在内部
                                cross_num = 0 # 交点数量清零,继续累计下一个
                            else: # 偶数个交点
                                flag_out = 0 # 点在外部
                                cross_num = 0 # 交点数量清零，继续累计下一个

                            if flag_in + flag_out == 0:
                                contact_flag = 1
                                break

                    if contact_flag == 1:
                        lowest_z[i,j] = -z[k]
                        break

            time.sleep(0.1)
            self.signal.emit(self.schedule)  # 发送实时任务进度


        """ 返回结果 """
        self.schedule = 100

        time.sleep(0.1)
        self.signal.emit(self.schedule)  # 发送实时任务进度

        X,Y = np.meshgrid(x,y)

        return X,Y,lowest_z
    
    def stop(self):
        self.is_running=False
        self.terminate()






