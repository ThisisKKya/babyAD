import config
from concurrent.futures import process
from scaner import *
import perception.perception as perception
from initial.initial import *
import planning.decision as planning
import scaner
# import control
#global
distanceData = perception.DistanceData()
# previous_distance =  DistanceData()
# current_distance = DistanceData()
MyCar = CarState()
VAR_Accelerator = 'AcceleratorAdditive'
VAR_Steer = 'AdditiveSteeringWheelAngle'
speedPidThread_1 = 10  # 控制阈值1
speedPidThread_2 = 2  # 控制阈值2

# 获取车辆左侧车道线index
def get_left_line_id(line_info):
    lines_count = Com_getShortData(line_info, 'linesNb0m')
    for i in range(lines_count):
        index = Com_getShortData(line_info, 'roadLinesInfosArray[{}]/position'.format(i))
        if index == -1:
            return i
    return 0

# 获取车辆右侧车道线index
def get_right_line_id(line_info):
    lines_count = Com_getShortData(line_info, 'linesNb0m')
    for i in range(lines_count):
        index = Com_getShortData(line_info, 'roadLinesInfosArray[{}]/position'.format(i))
        if index == 1:
            return i
    return 0

def latitudeyrControlpos(yr, yrPid):
    ''' 
    Lateral yr pid controller. use lateral angular velocity 
        to compensate lateral position control.

    Args:
        speed: car speed from ADCplatform.
        yrPid: lateral angular pid controller.
    
    Return: 
        None element
    '''
    yrPid.update(yr)
    yrPid.yrsteer_ = yrPid.output * -1


def latitudeControlpos(positionnow, latPid, MyCar):
    ''' 
    Lateral position pid controller to make the car 
        reach the target lane quickly and smoothly.

    Args:
        positionnow: car position now. 7 0 -7
        latPid: lateral position pid controller.
        MyCar: autonomous driving vehicle parameters, 
            CarState class type defined in initial.py
    
    Return: 
        None element
    '''
    latPid.update(positionnow)
    latPid.steer_ = latPid.output * -1.0
    if MyCar.speed > 20:
        latPid.steer_ = latPid.output * -0.8
    # print("lattel : ", latPid.steer_)
    # 缓慢变道尝试 可以但没必要 不利于提速
    # if abs(latPid.steer_) > 200:
    #     latPid.steer_ = 200 if latPid.steer_ > 0 else -200
    THRE_STEER = 60
    if abs(latPid.steer_) > THRE_STEER:
        latPid.steer_ = THRE_STEER if latPid.steer_ > 0 else - THRE_STEER

def lontitudeControlSpeed(speed, lonPid):
    ''' 
    Speed pid controller to make the car 
        reach the set speed quickly and brake.
        it includes five parts:
        stage 1 - speed
        stage 2 - keep speed
        stage 3 - finetune
        stage 4 - emergency brake
        stage 5 - brake finetune

    Args:
        speed: car speed from ADCplatform.
        lonPid: speed pid controller.
    
    Return: 
        None element
    '''
    lonPid.update(speed-5.0)
    if (lonPid.output > speedPidThread_1):    # 加速阶段
        # print('spezd is:', speed, 'output is:', lonPid.output, 'stage 1')
        lonPid.thorro_ = 1
        lonPid.brake_ = 0
    elif (lonPid.output > speedPidThread_2):  # 稳定控速阶段
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 2')
        lonPid.thorro_ = min((lonPid.output / speedPidThread_1) * 0.85, 1.0)
        lonPid.brake_ = min(((speedPidThread_1 - lonPid.output) / speedPidThread_1) * 0.1, 1.0)
    elif (lonPid.output > 0):                 # 下侧 微调
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 3')
        lonPid.thorro_ = (lonPid.output / speedPidThread_2) * 0.3
        # 0.5会有稍减速的效果40-38 防碰撞
        lonPid.brake_= ((speedPidThread_2 - lonPid.output) / speedPidThread_2) * 0.5
    elif (lonPid.output < -1 * speedPidThread_1):  # 减速一阶段
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 4')
        lonPid.thorro_ = (-1 * lonPid.output / 5) * 0.3
        # 减速第一阶段 仍然大于3m/s2 可选1.0 直接强制刹车
        lonPid.brake_= 1.0
    else :
        # print('speed is:', speed, 'output is:', lonPid.output, 'stage 5')
        lonPid.thorro_ = (-1 * lonPid.output / speedPidThread_2) * 0.15
        # 减速二阶段                 abs(2 - (2~10))/2 * 0.6
        # lonPid.brake_ = min(abs((speedPidThread_2 - (-1 * lonPid.output)) / speedPidThread_2) * 0.6, 1.0)
        lonPid.brake_ = 1.0
    # print(lonPid.thorro_, '    ', lonPid.brake_)

# 输入：目标交通信息接口，目标信息接口
# 输出：当前车道上最近的车辆的X方向距离(m)，若前方没有车辆，则输出float('inf')
def get_mid_distance(target_traffic_info, target_info):
    next_vehicle_in_lane_index = Com_getShortData(target_traffic_info, "nextVehicleInLane")
    if next_vehicle_in_lane_index != -1:
        # Process_OutputLevel("next_vehicle_in_lane_index = " + str(next_vehicle_in_lane_index), 4)
        # 计算当前车道上下一辆车的距离
        mid_distance = Com_getDoubleData(target_info, "targetsArray[{}]/posXInChosenRef".format(next_vehicle_in_lane_index))
        Process_OutputLevel("mid_distance = {}".format(mid_distance), 4)
        return mid_distance
    return float('inf')

# 输入：目标交通信息接口，目标信息接口
# 输出：左侧车道上最近的车辆的X方向距离(m)，若前方没有车辆，则输出float('inf')
def get_left_distance(target_traffic_info, target_info):
    target_array_count = Com_getShortData(target_info, "targetsArrayCount")
    if target_array_count == 0:
        return float('inf')

    left_distance = float('inf')
    for i in range(target_array_count):
        lane_id = Com_getShortData(target_traffic_info, "targetsArray[{}]/laneId".format(i))
        if lane_id == -1:
            temp_distance = Com_getDoubleData(target_info, "targetsArray[{}]/posXInChosenRef".format(i))
            # Process_OutputLevel("temp_distance = {}".format(temp_distance), 4)
            if temp_distance > 0 and temp_distance < left_distance:
                left_distance = temp_distance
    Process_OutputLevel("left_distance = {}".format(left_distance), 4)
    return left_distance

# 输入：目标交通信息接口，目标信息接口
# 输出：右侧车道上最近的车辆的X方向距离(m)，若前方没有车辆，则输出float('inf')
def get_right_distance(target_traffic_info, target_info):
    target_array_count = Com_getShortData(target_info, "targetsArrayCount")
    if target_array_count == 0:
        return float('inf')

    right_distance = float('inf')
    for i in range(target_array_count):
        lane_id = Com_getShortData(target_traffic_info, "targetsArray[{}]/laneId".format(i))
        if lane_id == 1:
            temp_distance = Com_getDoubleData(target_info, "targetsArray[{}]/posXInChosenRef".format(i))
            # Process_OutputLevel("temp_distance = {}".format(temp_distance), 4)
            if temp_distance > 0 and temp_distance < right_distance:
                right_distance = temp_distance
    Process_OutputLevel("right_distance = {}".format(right_distance), 4)
    return right_distance

def get_overtake_target_scaner_id(target_traffic_info, target_info):
    next_vehicle_in_lane_index = Com_getShortData(target_traffic_info, "nextVehicleInLane")
    if next_vehicle_in_lane_index != -1:
        # Process_OutputLevel("next_vehicle_in_lane_index = " + str(next_vehicle_in_lane_index), 4)
        # 计算被超对象ScanerId
        scaner_id = Com_getShortData(target_info, "targetsArray[{}]/scanerId".format(next_vehicle_in_lane_index))
        # Process_OutputLevel("mid_distance = {}".format(mid_distance), 4)
        return scaner_id
    return -1

def get_overtake_target_distance(target_info, scaner_id):
    target_array_count = Com_getShortData(target_info, "targetsArrayCount")
    for i in range(target_array_count):
        temp_id = Com_getShortData(target_info, "targetsArray[{}]/scanerId".format(i))
        if temp_id == scaner_id:
            return Com_getDoubleData(target_info, "targetsArray[{}]/posXInChosenRef".format(i))
    return 15

def main():
    #######初始化进程########
    parser = ScanerApiOption()
    (options, args) = parser.parse_args()
    Process_InitParams(options.process_name, options.configuration, options.frequency)

    ######定义&声明使用的数据接口#######
    # 获取车辆状态
    vehicle_info = Com_declareInputData("Network/IVehicle/VehicleUpdate", 0)

    # 车道线信息
    line_info = Com_declareInputData("Network/ISensor/RoadLinesPoints", 100000)
    line_poly_info = Com_declareInputData("Network/ISensor/RoadLinesPolynoms", 100000)

    # 车道信息
    # lane_info = Com_declareInputData('Network/IVehicle/VehiclePath', 100000)

    # 纵向速度控制
    cab_interface = Com_declareOutputData("Shm/ModelCabin/CabToModelCorrective", 0)
    CabToModel = Com_declareOutputData("Shm/ModelCabin/CabToModel", 0)
    # 转向控制
    steer_control = Com_declareOutputData("Shm/ModelCabin/CabToSteeringCorrective", 0)

    # 目标交通信息
    target_info = Com_declareInputData("Network/ISensor/SensorMovableTargets", 100000)
    target_traffic_info = Com_declareInputData("Network/ISensor/SensorMovableTargetsTrafficStates", 100000)

    controller = ControlData()
    # ego_vehicle_state = CarState()

    #######进程开始#####################################################################################
    status = PS_DAEMON
    try:
        while status != PS_DEAD:
            # Process manager Run
            Process_Run()
            Process_Wait()

            # Process manager State
            old_status = status
            status = Process_GetState()

            # Scaner API is now running
            if status == PS_RUNNING:
                Com_updateInputs(UT_AllData)

                Process_OutputLevel("-"*30, 4)
                # 计算左中右车道障碍物距离
                mid_distance = get_mid_distance(target_traffic_info, target_info)
                left_distance = get_left_distance(target_traffic_info, target_info)
                right_distance = get_right_distance(target_traffic_info, target_info)
                #############################percpetion 
                perception.run(distanceData,MyCar,right_distance,mid_distance,left_distance)
                #planning
                planning.run(distanceData, MyCar)

                # 获取车辆速度
                vehicle_speed = Com_getFloatData(vehicle_info, 'speed[0]')
                vehicle_heading = Com_getDoubleData(vehicle_info, 'pos[3]')
                vehicle_yr_speed = Com_getFloatData(vehicle_info, 'speed[4]')
                # vehicle_yr_speed = Com_getFloatData(vehicle_info, 'speed[5]')
                # Process_OutputLevel("speed = " + str(vehicle_speed), 4)
                
                # 获取车道线信息
                # lines_number = Com_getShortData(line_info, 'linesNb35m')
                # Process_OutputLevel("lines number = {}".format(lines_number), 4)
                left_line_coefficient = []
                right_line_coefficient = []
                for i in range(4):
                    left_line_coefficient.append(Com_getDoubleData(line_poly_info, 'roadLinesPolynomsArray[{}]/c{}'.format(get_left_line_id(line_info), i)))
                    right_line_coefficient.append(Com_getDoubleData(line_poly_info, 'roadLinesPolynomsArray[{}]/c{}'.format(get_right_line_id(line_info), i)))
                # left_c0 = Com_getDoubleData(line_poly_info, 'roadLinesPolynomsArray[{}]/c0'.format(get_left_line_id(line_info)))
                # right_c0 = Com_getDoubleData(line_poly_info, 'roadLinesPolynomsArray[{}]/c0'.format(get_right_line_id(line_info)))
                # Process_OutputLevel("left_c0 = {}".format(left_c0), 4)
                # Process_OutputLevel("right_c0 = {}".format(right_c0), 4)
                MyCar.speed = vehicle_speed
                MyCar.cao = vehicle_heading
                MyCar.yr = vehicle_yr_speed

                x = MyCar.lanefuture
                left_temp = left_line_coefficient[0] + left_line_coefficient[1]*x + left_line_coefficient[2]*x*x + left_line_coefficient[3]*x*x*x
                right_temp = right_line_coefficient[0] + right_line_coefficient[1]*x + right_line_coefficient[2]*x*x + right_line_coefficient[3]*x*x*x
                # Process_OutputLevel("position now = {}".format(MyCar.positionnow), 4)
                MyCar.positionnow =  (left_temp + right_temp) * 1.75 
                
                # 有限3种状态任务
                Process_OutputLevel("Current state: " + MyCar.cardecision, 4)
                Process_OutputLevel("direction: {}".format(MyCar.direction), 4)
                Process_OutputLevel("MyCar.changing = {}".format(MyCar.changing) ,4)
                # Process_OutputLevel("MyCar.lanestate = ({}, {}, {})".format(MyCar.lanestate.LEFT, MyCar.lanestate.MID, MyCar.lanestate.RIGHT), 4)
                # Com_setDoubleData(cab_interface,VAR_Accelerator,1)
                # Com_setDoubleData(cab_interface,'AcceleratorMultiplicative', 0)
                Process_OutputLevel("midlane = {}".format(MyCar.midlane), 4)
                if (MyCar.cardecision == 'overtake'):
                    controller.speedPid.setSetpoint(controller.overtakelimit)
                    # 纵向控制 thorro_ and brake_
                    lontitudeControlSpeed(MyCar.speed, controller.speedPid)
                    # overtake 车道中线调整
                    if (not MyCar.changing):
                        # 最左侧不可左变道
                        if (MyCar.direction == 'left'):
                            MyCar.midlane = min(MyCar.lanestate.LEFT , MyCar.lanestate.LEFT + MyCar.midlane)
                        # 最右侧不可右变道
                        elif (MyCar.direction == 'right'):
                            MyCar.midlane = max(MyCar.lanestate.RIGHT , MyCar.lanestate.RIGHT + MyCar.midlane)
                        controller.latPid.setSetpoint(MyCar.midlane)
                        # 更新中线state 进入超车
                        MyCar.changing = True
                        scaner_id = get_overtake_target_scaner_id(target_traffic_info, target_info)
                    # Process_OutputLevel("MyCar.changing2 = {}".format(MyCar.changing) ,4)
                    # overtake 完成 切换 follow 状态跟车
                    # print("minus : ", MyCar.midlane - MyCar.positionnow)
                    # if (MyCar.changing and abs(MyCar.midlane - MyCar.positionnow) < 0.5):
                    
                    # if (MyCar.changing and 
                    #     (distanceData.distance_mid > MyCar.saftydistance + 5
                    #     or abs(MyCar.midlane - MyCar.positionnow) < 0.5)
                    #     ):
                    overtake_target_distance = get_overtake_target_distance(target_info, scaner_id)
                    Process_OutputLevel("overtake_target_distance = {}".format(overtake_target_distance), 4)
                    if (overtake_target_distance <= 5):
                        MyCar.cardecision = 'speedup'
                        MyCar.direction = 'mid'
                        MyCar.changing = False
                        MyCar.overtakeSum += 1
                    # Process_OutputLevel("MyCar.changing3 = {}".format(MyCar.changing) ,4)
                    # 横向控制 steer_ 加入角度速度约束
                    latitudeyrControlpos(MyCar.yr, controller.yrPid)
                    # print('yr is', MyCar.yr, 'steeryr is', Controller.yrPid.yrsteer_) # overtake >15 , normal < 3
                    # print('latsteer is ', Controller.latPid.steer_)
                    latitudeControlpos(MyCar.positionnow, controller.latPid, MyCar)

                    Com_setDoubleData(cab_interface, VAR_Accelerator,controller.speedPid.thorro_)
                    # Com_setDoubleData(cab_interface,VAR_Accelerator,20)
                    # Com_setDoubleData(cab_interface,VAR_Accelerator,0)
                    Com_setDoubleData(steer_control, VAR_Steer,controller.latPid.steer_ + 0.01 * controller.yrPid.yrsteer_)
                    # Com_setShortData(CabToModel, 'IgnitionKey', 2)
                    # Com_setShortData(CabToModel, 'GearBoxAutoMode', 10)
                    Com_setDoubleData(CabToModel,'Brake',controller.speedPid.brake_)

                elif (MyCar.cardecision == 'speedup'):
                    if MyCar.time >= controller.superspeeduplimittime \
                        and MyCar.overtakeSum != 0:
                        controller.speeduplimit = controller.superspeeduplimit
                    controller.speedPid.setSetpoint(controller.speeduplimit)
                    # 纵向控制 thorro_ and brake_
                    lontitudeControlSpeed(MyCar.speed, controller.speedPid)
                    # 横向控制 steer_
                    latitudeControlpos(MyCar.positionnow, controller.latPid, MyCar)
                    # Com_setDoubleData(cab_interface,VAR_Accelerator,20)
                    Com_setDoubleData(cab_interface, VAR_Accelerator, controller.speedPid.thorro_)
                    # Process_OutputLevel("controller.speedPid.thorro_ = {}".format(controller.speedPid.thorro_), 4)

                    Com_setDoubleData(cab_interface,'AcceleratorMultiplicative',0)
                    # Com_setShortData(CabToModel, 'IgnitionKey', 2)
                    # Com_setShortData(CabToModel, 'GearBoxAutoMode', 10)
                    Com_setDoubleData(steer_control, VAR_Steer, controller.latPid.steer_)
                    # Process_OutputLevel("controller.latPid.steer_ = {}".format(controller.latPid.steer_), 4)
                    Com_setDoubleData(CabToModel, 'Brake', controller.speedPid.brake_)
                    # Process_OutputLevel("controller.speedPid.brake_ = {}".format(controller.speedPid.brake_), 4)
                    
                elif (MyCar.cardecision == 'follow'):
                    controller.speedPid.setSetpoint(controller.followlimit)
                    # 纵向控制 thorro_ and brake_
                    lontitudeControlSpeed(MyCar.speed, controller.speedPid)
                    # 横向控制 steer_
                    latitudeControlpos(MyCar.positionnow, controller.latPid, MyCar)
                    # # Com_setDoubleData(cab_interface,VAR_Accelerator,controller.speedPid.thorro_)
                    Com_setDoubleData(cab_interface,VAR_Accelerator,20)
                    Com_setDoubleData(cab_interface,VAR_Accelerator,0)
                    Com_setShortData(CabToModel, 'IgnitionKey', 2)
                    Com_setShortData(CabToModel, 'GearBoxAutoMode', 10)

                    Com_setDoubleData(steer_control,VAR_Steer,controller.latPid.steer_)
                    Com_setDoubleData(CabToModel,'Brake',controller.speedPid.brake_)
                Com_updateOutputs(UT_AllData)

    except KeyboardInterrupt:
        print('Bye bye')
        Process_Close()


if __name__ == '__main__':
    main()

