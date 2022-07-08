import control.pid as pid
class LaneState(object):
    """
    The lane state class using lateral control
    A target number of driving lane, which is the sum of two dashed lane line.
    
    Attributes:
        LEFT: left lane target, float type
        MID: middle lane target, float type
        RIGHT: right lane target, float type
    """
    
    def __init__(self, l=7.5, m=0.0, r=-7.5):
        """ Initializing three lane target number """
        self.LEFT = l
        self.MID = m
        self.RIGHT = r
    
    def set_leftlane(self, l):
        self.LEFT = l
    
    def set_midlane(self, m):
        self.MID = m
    
    def set_rightlane(self, r):
        self.RIGHT = r

class ControlData(object):
    """
    车辆控制算法相关参数
    """
    
    def __init__(self):

        self.speeduplimit = 20               # 加速阶段控制速度
        self.superspeeduplimit = 25         # 超级加速阶段控制速度
        self.superspeeduplimittime = 45      # 超级加速阶段计时阈值
        self.followlimit = 20                # 跟车阶段控制速度
        self.overtakelimit = 25              # 超车阶段控制速度
        
        # 横向控制PID参数
        self.lat_kp = 0.75
        self.lat_ki = 0.02
        self.lat_kd = 0.4
        # self.lat_kp = 100
        # self.lat_ki = 0.02
        # self.lat_kd = 1
        self.latPid = pid.PID(self.lat_kp, self.lat_ki, self.lat_kd)
        
        # 方向角控制PID参数
        self.yr_kp = 0.06
        self.yr_ki = 0.01
        self.yr_kd = 0.05
        # self.yr_kp = 100
        # self.yr_ki = 0.01
        # self.yr_kd = 0.2     
        self.yrPid = pid.PID(self.yr_kp, self.yr_ki, self.yr_kd)
        
        # 纵向控制PID参数
        self.targetSpeedInit = 20.0          # 想要到达的速度
        self.speed_kp = 1.20
        self.speed_ki = 0.02
        self.speed_kd = 0.5
        self.speedPid = pid.PID(self.speed_kp, 0, self.speed_kp)
        self.speedPidThread_1 = 10
        self.speedPidThread_2 = 2

    def initPID(self):
        self.speedPid.clear() # lon
        self.latPid.clear()   # lat
        self.yrPid.clear()    # lat
        self.speedPid.setSetpoint(self.targetSpeedInit)
        self.latPid.setSetpoint(0)             # lat aim 0
        self.yrPid.setSetpoint(0)              # lat aim 0

class CarState(object):
    """
    车辆状态和车辆行驶算法相关的标志位与参数
    """
    
    def __init__(self):
        self.lanestate = LaneState(7, 0, -8) # 左中右车道线目标位置
        self.speed = 0                       # 车辆当前速度
        self.cao = 0                         # 车辆当前姿态
        self.yr = 0                          # 车辆当前角速度
        self.positionnow = 0                 # 两车道线A1求和
        
        self.cardecision = 'speedup'         # planning计算得到决策
        self.direction = 'mid'               # 当前行驶方向
        self.changing = False                # 处于超车状态时为True
        self.midlane = self.lanestate.MID    # 7.5 0 -8 latpid 参考 target
        self.lanefuture = 2.0                # 车道线 x = 2 处的位置
        self.saftydistance = 40             # 与前车的安全距离 对于紧密跟车的情况 要准确识别并控速
        self.lastovertakeSum = 0             # 超车计数与数据平滑辅助变量
        self.overtakeSum = 0                 # 超车计数
        self.time = 0                        # 超级加速阶段计时
        self.finalflag = False               # 超级加速阶段回到中间车道标志位
        
        # # Initilize three triangles using perception
        # self.triangle_mid = Triangle(240, 175, 160, 250, 320, 250)
        # self.triangle_left = Triangle(240, 175, 160, 250, -70, 280)
        # self.triangle_right = Triangle(250, 175, 550, 280, 320, 250)