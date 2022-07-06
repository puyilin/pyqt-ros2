import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import style
import os
from mpl_toolkits.mplot3d import Axes3D
from ARIE import ARIE_formulation

os.system('cls')

if __name__ == '__main__':

    """ 创建ARIE构建对象 """
    arie = ARIE_formulation(2,30,0,0) # 传入半径 r 和三个轴转角: theta_x,theta_y,theta_z
    X,Y,lowest_z = arie.ARIE_calculation() # 调用计算函数

    """ 图窗设置 """
    matplotlib.rcParams['text.usetex'] = True # 开启latex风格

    # fig = plt.figure(figsize=(3,2),dpi=70) # 设置图像大小

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot_surface(X,Y,lowest_z,cmap='rainbow')
    ax.set_xlabel('x', fontsize = 10)
    ax.set_ylabel('y', fontsize = 10)
    ax.set_zlabel('z', fontsize = 10)

    plt.show()