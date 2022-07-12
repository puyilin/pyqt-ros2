class tsakrequirements():
    
    def __init__(self, radius):
        # 轴的参数初始化
        self.radius = 2.0
        # 轴的初始偏转
        self.p_init_ax = 30
        self.p_init_ay = 0
        self.p_init_az = 0
        # 轴的初始位置
        self.p_init_px = 0.0
        self.p_init_py = 0.0
        self.p_init_pz = 0.0
        # 吸引域最低点
        self.arie_lowest_x = 0
        self.arie_lowest_y = 0
        self.arie_lowest_z = 0
        # 孔的位置
        self.h_init_px = 0
        self.h_init_py = 0 
        self.h_init_pz = 0

    def send_radius(self, radius):

        self.radius = radius
        return self.radius

    def send_request_a(self, ax, ay, az):

        self.p_init_ax = ax
        self.p_init_ay = ay
        self.p_init_az = az
        return self.p_init_ax, self.p_init_ay, self.p_init_az

    def send_request_s(self, px, py, pz):

        self.p_init_px = px
        self.p_init_py = py
        self.p_init_pz = pz
        return self.p_init_px, self.p_init_py, self.p_init_pz

    def send_lowest_p(self, lowest_px, lowest_py, lowest_pz):

        self.arie_lowest_x = lowest_px
        self.arie_lowest_y = lowest_py
        self.arie_lowest_z = lowest_pz
        return self.arie_lowest_x, self.arie_lowest_y, self.arie_lowest_z

    def send_h_init_p(self, coordinate):
        self.h_init_px, self.h_init_py, self.h_init_pz = coordinate[:]