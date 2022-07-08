import sys
sys.path.append('D:\software\\2022CICV\SCANeRstudio-2022.1r34_Trial\SCANeRstudio_2022\APIs\samples\ScanerAPI\python\\')
from scaner import *

#######初始化进程########
parser = ScanerApiOption()
(options, args) = parser.parse_args()
Process_InitParams(options.process_name, options.configuration, options.frequency)

######定义&声明使用的数据接口#######
Longitudinal_control_Interface = str("Shm/ModelCabin/CabToModelCorrective")
Longitudinal_control = Com_declareOutputData(Longitudinal_control_Interface, 0)
Cab_interface = Com_declareOutputData("Shm/ModelCabin/CabToModel", 0)

#######进程开始########
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
           # Com_updateInputs(UT_AllData)
            Com_setDoubleData(Longitudinal_control,'AcceleratorMultiplicative',0)
            Com_setDoubleData(Longitudinal_control, 'AcceleratorAdditive', 1)
            #Com_setCharData(Longitudinal_control,'GearboxTakeOver','true')
            Com_setShortData(Cab_interface, 'IgnitionKey', 2)
            Com_setShortData(Cab_interface, 'GearBoxAutoMode', 10)
            Com_updateOutputs(UT_AllData)


except KeyboardInterrupt:
    print('Bye bye')
    Process_Close()



