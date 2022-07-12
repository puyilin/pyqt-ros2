from Taskrequirement import tsakrequirements
from rclpy.node import Node
from ARIE import ARIE_formulation
from perception import perception_node


class StrategiesModules(Node):

    def __init__(self):

        r = 20
        self.p_init_px, self.p_init_py, self.p_init_pz = tsakrequirements.send_request_s()
        self.p_init_ax, self.p_init_ay, self.p_init_az = tsakrequirements.send_request_a()
        arie = ARIE_formulation(r, self.p_init_px, self.p_init_py, self.p_init_pz)
        self.arie_lowest_x, self.arie_lowest_y, self.arie_lowest_z = arie.ARIE_calculation()
        self.h_init_px, self.h_init_py, self.h_init_pz = perception_node.coordinate_get()

    def strategies_formulation(self):

        # 策略分为4个阶段
        # 一阶段：从初始位置下落到吸引域对应的点；二阶段：从对应点滑到吸引域最低点
        # 三阶段：主动偏转一定角度；四阶段：竖直下降进入孔内

        # 阶段1 the target_position of phase_1
        target_px = format(self.p_init_px,'.2f')
        target_py = format(self.p_init_py,'.2f')
        target_pz = format(self.arie_lowest_z + self.h_init_pz,'.2f') # 从 初始位置 下降到 吸引域边界对应的点 
        target_ax = self.p_init_ax 
        target_ay = self.p_init_ay 
        target_az = self.p_init_az # 姿态保持不变
        target_phase_1 = str(target_px) +' : '+ str(target_py) +' : '+ str(target_pz) +' : '+ str(target_ax) +' : '+ str(target_ay) +' : '+ str(target_az)
        self.get_logger().info('The desired position and orientation of the peg in the phase one: ' + target_phase_1)
        constraint_phase_1 = '0' +' : '+ '0' +' : '+ '0' +' : '+ '1' +' : '+ '1' +' : '+ '1' # xyz三个方向位置放松，xyz角度约束
        self.get_logger().info('The constraint flag of the movement and rotation as to xyz-axes in the phase one: ' + constraint_phase_1)

        # 阶段2 the target position of phase_2
        target_px = format(self.arie_lowest_x + self.h_init_px,'.2f')
        target_py = format(self.arie_lowest_y + self.h_init_py,'.2f')
        target_pz = format(self.arie_lowest_z + self.h_init_pz,'.2f') # 从 吸引域边界对应点 到 吸引域最低点
        target_ax = self.p_init_ax
        target_ay = self.p_init_ay
        target_az = self.p_init_az # 姿态保持不变
        target_phase_2 = str(target_px) +' : '+ str(target_py) +' : '+ str(target_pz) +' : '+ str(target_ax) +' : '+ str(target_ay) +' : '+ str(target_az)
        self.get_logger().info('The desired position and orientation of the peg in the phase two: ' + target_phase_2)
        constraint_phase_2 = '0' +' : '+ '0' +' : '+ '0' +' : '+ '1' +' : '+ '1' +' : '+ '0' # xyz三个方向位置放松，xy角度约束，z角度放松
        self.get_logger().info('The constraint flag of the movement and rotation as to xyz-axes in the phase two: ' + constraint_phase_2)

        # 阶段3 the target position of phase_3 
        target_px = format(self.arie_lowest_x + self.h_init_px,'.2f')
        target_py = format(self.arie_lowest_y + self.h_init_py,'.2f')
        target_pz = format(self.arie_lowest_z + self.h_init_pz,'.2f') # 在吸引域最低点 位置保持不变
        if self.p_init_ax != 0 :
            target_ax = 0  # 姿态角开始偏转 到 各轴夹角为0
        else: 
            target_ax = self.p_init_ax
        if self.p_init_ay != 0 :
            target_ay = 0  # 姿态角开始偏转 到 各轴夹角为0
        else: 
            target_ay = self.p_init_ay
        if self.p_init_az != 0 :
            target_az = 0  # 姿态角开始偏转 到 各轴夹角为0
        else: 
            target_az = self.p_init_az
        target_phase_3 = str(target_px) +' : '+ str(target_py) +' : '+ str(target_pz) +' : '+ str(target_ax) +' : '+ str(target_ay) +' : '+ str(target_az)
        self.get_logger().info('The desired position and orientation of the peg in the phase three: ' + target_phase_3)
        constraint_phase_3 = '0' +' : '+ '0' +' : '+ '0' +' : '+ '1' +' : '+ '1' +' : '+ '0' # xyz三个方向位置放松，xy角度约束，z角度放松
        self.get_logger().info('The constraint flag of the movement and rotation as to xyz-axes in the phase three: ' + constraint_phase_3)

        # 阶段4 the target position of phase_4
        target_px = format(self.arie_lowest_x + self.h_init_px,'.2f')
        target_py = format(self.arie_lowest_y + self.h_init_py,'.2f')
        target_pz = format(self.arie_lowest_z + self.h_init_pz - 30,'.2f') # 从 吸引域最低点 到 下沉到孔中 -30mm
        target_ax = 0
        target_ay = 0 
        target_az = 0 # 各轴夹角为0 保持不变
        target_phase_4 = str(target_px) +' : '+ str(target_py) +' : '+ str(target_pz) +' : '+ str(target_ax) +' : '+ str(target_ay) +' : '+ str(target_az)
        self.get_logger().info('The desired position and orientation of the peg in the phase four: ' + target_phase_4)
        constraint_phase_4 = '0' +' : '+ '0' +' : '+ '0' +' : '+ '1' +' : '+ '1' +' : '+ '0' # xyz三个方向位置放松，xy角度约束，z角度放松
        self.get_logger().info('The constraint flag of the movement and rotation as to xyz-axes in the phase four: ' + constraint_phase_4)