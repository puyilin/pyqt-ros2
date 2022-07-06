import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import style
import time
import math
import os
from mpl_toolkits.mplot3d import Axes3D

os.system('cls')

''' 参数设置 '''

# 零件尺寸
R=2
H=2

# 分辨率
resolution = np.array([90,90,100])

# 设置n边形顶点坐标
vertex_num = 99
vertex_position_initial = np.zeros([4,vertex_num*2])
theta = np.linspace(0,2*math.pi,num=vertex_num)
vertex_position_initial[0,0:vertex_num] = R*np.cos(theta)
vertex_position_initial[1,0:vertex_num] = R*np.sin(theta)
vertex_position_initial[2,0:vertex_num] = 0
vertex_position_initial[3,0:vertex_num] = 1
vertex_position_initial[:,vertex_num:vertex_num*2] = vertex_position_initial[:,0:vertex_num]

# 初始转角设置 alpha:rotation around x-axis
alpha = 0/180*math.pi
beta = 30/180*math.pi
gamma = 0/180*math.pi

# print(vertex_position_initial[1,:])

'''数据计算'''

# 旋转矩阵
rotate_x = np.array([
    [1,0,0,0],
    [0,math.cos(alpha),-math.sin(alpha),0],
    [0,math.sin(alpha),math.cos(alpha),0],
    [0,0,0,1]])
rotate_y = np.array([
    [math.cos(beta),0,math.sin(beta),0],
    [0,1,0,0],
    [-math.sin(beta),0,math.cos(beta),0],
    [0,0,0,1]])
rotate_z = np.array([
    [math.cos(gamma),-math.sin(gamma),0,0],
    [math.sin(gamma),math.cos(gamma),0,0],
    [0,0,1,0],
    [0,0,0,1]])

# Tt = (rotate_x * rotate_y) * rotate_z '*' 是点乘!
Tt = np.dot(np.dot(rotate_x,rotate_y),rotate_z)
Tt[0:3,3] = np.array([0,0,0]).T

vertex_position_current = np.zeros([4,vertex_num*2])
vertex_position_current[:,0:vertex_num] = np.dot(Tt,vertex_position_initial[:,0:vertex_num])
vertex_position_current[:,vertex_num:vertex_num*2] = vertex_position_current[:,0:vertex_num]

# print(Tt)
# print(vertex_position_current[2,0:vertex_num])

# 方向向量
vector_dir = np.dot(Tt,np.array([0,0,H,0]).T)

min_init_x = np.min(vertex_position_initial[0,0:vertex_num])
max_init_x = np.max(vertex_position_initial[0,0:vertex_num])
min_init_y = np.min(vertex_position_initial[1,0:vertex_num])
max_init_y = np.max(vertex_position_initial[1,0:vertex_num])

index_min_init_x = np.argmin(vertex_position_initial[0,0:vertex_num])
index_max_init_x = np.argmax(vertex_position_initial[0,0:vertex_num])
index_min_init_y = np.argmin(vertex_position_initial[1,0:vertex_num])
index_max_init_y = np.argmax(vertex_position_initial[1,0:vertex_num])

# 旋转标志 1(+) & 2(-) for rotate x-axis, 3(+) & 4(-) for ratate y-axis
flag_rotation = 0

# 平移因子
scale_buff = 0.5

temp_b = np.min(vertex_position_current[2,0:vertex_num])

if beta == 0 and alpha > 0: # 绕x轴正转
    temp_a = np.min(vertex_position_current[1,0:vertex_num])# temp_a<0
    x = np.linspace(min_init_x-scale_buff,max_init_x+scale_buff,num=resolution[0])
    y = np.linspace(min_init_y-temp_a-scale_buff,max_init_y-temp_a+scale_buff,num=resolution[1])
    z = np.linspace(temp_a*math.tan(alpha),-temp_b,num=resolution[2])
    flag_rotation = 1
elif beta == 0 and alpha < 0: # 绕x轴反转
    temp_a = np.max(vertex_position_current[1,0:vertex_num])# temp_a>0
    x = np.linspace(min_init_x-scale_buff,max_init_x+scale_buff,num=resolution[0])
    y = np.linspace(min_init_y-temp_a-scale_buff,max_init_y-temp_a+scale_buff,num=resolution[1])
    z = np.linspace(temp_a*math.tan(alpha),-temp_b,num=resolution[2])
    flag_rotation = 2    
elif alpha == 0 and beta > 0: # 绕y轴正转
    temp_a = np.max(vertex_position_current[0,0:vertex_num])# temp_a>0
    x = np.linspace(min_init_x-temp_a-scale_buff,max_init_x-temp_a+scale_buff,num=resolution[0])
    y = np.linspace(min_init_y-scale_buff,max_init_y+scale_buff,num=resolution[1])
    z = np.linspace(-temp_a*math.tan(beta),-temp_b,num=resolution[2])
    flag_rotation = 3  
elif alpha == 0 and beta < 0: # 绕y轴反转
    temp_a = np.min(vertex_position_current[0,0:vertex_num])# temp_a>0
    x = np.linspace(min_init_x-temp_a-scale_buff,max_init_x-temp_a+scale_buff,num=resolution[0])
    y = np.linspace(min_init_y-scale_buff,max_init_y+scale_buff,num=resolution[1])
    z = np.linspace(-temp_a*math.tan(beta),-temp_b,num=resolution[2])
    flag_rotation = 4
else:
    print('Please rotate around one axis!')  

length_x = len(x); length_y = len(y); length_z = len(z)

lowest_z = np.zeros([length_x,length_y])

# print(z)

""" 开始计算 """

for i in range(length_x):
    for j in range(length_y):

        print(i)
        cross_line_num = 0 # 与穿过直线交点的个数
        flag_offset = 0 # 偏置标志

        if flag_rotation == 1:
            for m in range(vertex_num):
                if vertex_position_initial[1,m] - temp_a > y[j] and \
                ((vertex_position_initial[0,m] < x[i] and vertex_position_initial[0,m+1] > x[i]) or \
                (vertex_position_initial[0,m] > x[i] and vertex_position_initial[0,m+1] < x[i])):
                    cross_line_num += 1
        elif flag_rotation == 2:
            for m in range(vertex_num):
                if vertex_position_initial[1,m] - temp_a > y[j] and \
                    ((vertex_position_initial[0,m] < x[i] and vertex_position_initial[0,m+1] > x[i]) or \
                    (vertex_position_initial[0,m] > x[i] and vertex_position_initial[0,m+1] < x[i])):
                    cross_line_num += 1
        elif flag_rotation == 3:
            for m in range(vertex_num):
                if vertex_position_initial[1,m] > y[j] and \
                    ((vertex_position_initial[0,m] - temp_a < x[i] and vertex_position_initial[0,m+1] - temp_a > x[i]) or \
                    (vertex_position_initial[0,m] - temp_a > x[i] and vertex_position_initial[0,m+1] - temp_a < x[i])):
                    cross_line_num += 1
        elif flag_rotation == 4:
            for m in range(vertex_num):
                if vertex_position_initial[1,m] > y[j] and \
                    ((vertex_position_initial[0,m] - temp_a < x[i] and vertex_position_initial[0,m+1] - temp_a > x[i]) or \
                    (vertex_position_initial[0,m] - temp_a > x[i] and vertex_position_initial[0,m+1] - temp_a < x[i])):
                    cross_line_num += 1
        
        for k in range(length_z):

            if cross_line_num % 2 == 0:
                lowest_z[i,j] = -z[k]
                break
        
            Tt[0:3,3] = np.array([x[i],y[j],-z[k]]).T

            vertex_position_current[:,0:vertex_num] = np.dot(Tt,vertex_position_initial[:,0:vertex_num])
            vertex_position_current[:,vertex_num:vertex_num*2] = vertex_position_current[:,0:vertex_num]
            vector_dir = np.dot(Tt,np.array([0,0,H,0]).T) # 方向向量

            # 计算切面
            section_num = 0
            section_position = np.zeros([4,vertex_num+1])
            section_position[3,:] = 1
            under_flag = 0 ; under_index = 0

            for temp_i in range(vertex_num):
                if vertex_position_current[2,temp_i] < 0:
                    under_flag = 1
                    under_index = temp_i
                    break

            if under_flag == 1:

                for temp_i in range(vertex_num):
                    temp_j = temp_i + under_index
                    if vertex_position_current[2,temp_j] <= 0:
                        section_position[0:2,section_num] = vertex_position_current[0:2,temp_j] - \
                            (vertex_position_current[2,temp_j]/vector_dir[2])*vector_dir[0:2]
                        section_num += 1    
                    else:
                        if vertex_position_current[2,temp_j-1] < 0:
                            section_position[0:2,section_num] = vertex_position_current[0:2,temp_j-1] + \
                                (-vertex_position_current[2,temp_j-1]/(vertex_position_current[2,temp_j]-vertex_position_current[2,temp_j-1])) * \
                                (vertex_position_current[0:2,temp_j] - vertex_position_current[0:2,temp_j-1])
                            section_num += 1
                        elif vertex_position_current[2,temp_j+1] < 0:
                            section_position[0:2,section_num] = vertex_position_current[0:2,temp_j] + \
                                (-vertex_position_current[2,temp_j]/(vertex_position_current[2,temp_j+1]-vertex_position_current[2,temp_j])) * \
                                (vertex_position_current[0:2,temp_j+1] - vertex_position_current[0:2,temp_j])
                            section_num += 1  

            # 判断是否接触
            contact_flag = 0; flag_out = 1; flag_in = 1; temp = 0; cross_num = 0   

            if under_flag == 1:

                for temp_i in range(section_num):
                    for temp_j in range(vertex_num):
                        if vertex_position_initial[1,temp_j] >= section_position[1,temp_i] and \
                            ((vertex_position_initial[0,temp_j] < section_position[0,temp_i] and vertex_position_initial[0,temp_j+1] > section_position[0,temp_i]) or \
                            (vertex_position_initial[0,temp_j] > section_position[0,temp_i] and vertex_position_initial[0,temp_j+1] < section_position[0,temp_i]) or \
                            (vertex_position_initial[0,temp_j] == section_position[0,temp_i])):
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

""" 画图 """

X,Y = np.meshgrid(x,y)

# 图窗设置
matplotlib.rcParams['text.usetex'] = True # 开启latex风格

# fig = plt.figure(figsize=(3,2),dpi=70) # 设置图像大小

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot_surface(X,Y,lowest_z,cmap='rainbow')
ax.set_xlabel('x', fontsize = 10)
ax.set_ylabel('y', fontsize = 10)
ax.set_zlabel('z', fontsize = 10)

plt.show()