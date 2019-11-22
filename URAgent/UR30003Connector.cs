using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Net;
using System.Reflection;
using LogPrinter;
using MathFunction;
using System.Diagnostics;
namespace URCommunication
{
    /// <summary>
    /// UR中的30003端口通讯类
    /// </summary>
    public class UR30003Connector : URTCPBase
    {
        #region 枚举
        /// <summary>
        /// 实时参数协议格式首位
        /// </summary>
        protected enum RealTimeDatasMarks : int
        {
            Size = 0,
            Time = 4,
            Targetq = 12,
            Targetqd = 60,
            Targetqdd = 108,
            TargetI = 156,
            TargetT = 204,
            Actualq = 252,
            Actualqd = 300,
            ActualI = 348,
            ControlI = 396,
            Actualp = 444,
            Actualpd = 492,
            TCPF = 540,
            Targetp = 588,
            Targetpd = 636,
            DigitalIn = 684,
            Temperature = 692,
            ControlT = 740,
            ModeR = 756,
            ModeJ = 764,
            ModeS = 812,
            ToolAcceleration = 868,
            SpeedScaling = 940,
            TorqueNorm = 948,
            VoltageMain = 972,
            VoltageRobot = 980,
            CurrentRobot = 988,
            VoltageJ = 996,
            DigitalOut = 1044,
            StateProg = 1052,
            ElbowPos = 1060,
            ElbowV = 1084
        }

        /// <summary>
        /// 机械臂状态
        /// </summary>
        public enum RobotStatus : short
        {
            PowerOff = 0,
            PowerOn,
            Idle,
            Running
        }

        /// <summary>
        /// 机械臂紧急状态
        /// </summary>
        public enum RobotEmergency : short
        {
            ProtectiveStop = 0,
            EmergencyStop,
            CurrentOverflow,
            ForceOverflow
        }

        /// <summary>
        /// 机器臂程序运行状态
        /// </summary>
        public enum RobotProgramStatus : short
        {
            Begin = 0,
            Idle,
            Running,
            Pausing,
            Paused
        }

        /// <summary>
        /// 机器人程序类型
        /// </summary>
        public enum RobotProgramType : byte
        {
            SW34 = 0,
            SW35
        }

        /// <summary>
        /// 力信号修正方式
        /// </summary>
        public enum ForceModifiedMode : byte
        {
            NoModified = 0,
            ProbePrecise = 1,
            PuncturePrecise = 2,
            ProbeWhole = 3
        }
        #endregion

        #region 字段 30003端口参数 详见手册
        protected int totalLength = 0;

        protected int robotTimeHour = 0;
        protected int robotTimeMinute = 0;
        protected int robotTimeSecond = 0;
        protected int robotTimeMicroSecond = 0;

        protected double[] positionsJointTarget = new double[6];
        protected double[] speedsJointTarget = new double[6];
        protected double[] accelerationsJointTarget = new double[6];
        protected double[] currentsJointTarget = new double[6];
        protected double[] torquesJointTarget = new double[6];

        protected double[] positionsJointActual = new double[6];
        protected double[] speedsJointActual = new double[6];
        protected double[] currentsJointActual = new double[6];
        protected double[] currentsJointControl = new double[6];

        protected double[] positionsTcpActual = new double[6];
        protected double[] speedsTcpActual = new double[6];
        protected double[] forcesTcpActual = new double[6];

        protected double[] positionsTcpTarget = new double[6];
        protected double[] speedsTcpTarget = new double[6];

        protected bool[] inputDigital = new bool[8];
        protected bool[] inputConfig = new bool[8];
        protected bool[] inputTool = new bool[2];

        protected double[] temperaturesJoint = new double[6];

        protected double controllerTimer = 0.0;

        protected bool modeRobotDisconnected = false;
        protected bool modeRobotConfirmSafety = false;
        protected bool modeRobotBooting = false;
        protected bool modeRobotPowerOff = false;
        protected bool modeRobotPowerOn = false;
        protected bool modeRobotIdle = false;
        protected bool modeRobotBackDrive = false;
        protected bool modeRobotRunning = false;
        protected bool modeRobotUpdatingFirmware = false;

        protected bool[] modeJointsShuttingdown = new bool[6];
        protected bool[] modeJointsPartDCalibration = new bool[6];
        protected bool[] modeJointsBackDrive = new bool[6];
        protected bool[] modeJointsPowerOff = new bool[6];
        protected bool[] modeJointsNotResponding = new bool[6];
        protected bool[] modeJointsMotorInitialisation = new bool[6];
        protected bool[] modeJointsBooting = new bool[6];
        protected bool[] modeJointsPartDCalibrationError = new bool[6];
        protected bool[] modeJointsBootLoader = new bool[6];
        protected bool[] modeJointsCalibration = new bool[6];
        protected bool[] modeJointsFault = new bool[6];
        protected bool[] modeJointsRunning = new bool[6];
        protected bool[] modeJointsIdle = new bool[6];

        protected bool modeSafetyNormal = false;
        protected bool modeSafetyReduced = false;
        protected bool modeSafetyProtectiveStop = false;
        protected bool modeSafetyRecovery = false;
        protected bool modeSafetySafeGuardStop = false;
        protected bool modeSafetySystemEmergencyStop = false;
        protected bool modeSafetyRobotEmergencyStop = false;
        protected bool modeSafetyViolation = false;
        protected bool modeSafetyFault = false;

        protected double[] accelerationsTcpActual = new double[3];
        protected double speedScaling = 0.0;
        protected double momentumTcpActual = 0.0;

        protected double voltageMain = 0.0;
        protected double voltageRobot = 0.0;
        protected double currentRobot = 0.0;
        protected double[] voltagesJoint = new double[6];

        protected bool[] outputDigital = new bool[8];
        protected bool[] outputConfig = new bool[8];
        protected bool[] outputTool = new bool[2];

        protected double programState = 0.0;

        protected double[] elbowPosition = new double[3];
        protected double[] elbowVelocity = new double[3];
        #endregion

        #region 字段 49152力传感器端口参数 修正
        protected double[] originalFlangeForces = new double[6];
        protected double[] zeroedOriginalFlangeForces = new double[6];
        protected double[] removeGravityFlangeForces = new double[6];
        protected double[] removeGravityBaseForces = new double[6];
        protected double[] removeGravityTcpForces = new double[6];
        #endregion

        #region 字段 额外附加的部分参数
        protected double[] positionsFlangeActual = new double[6]; // 法兰在基系坐标

        protected List<double[]> continuousTcpPositions = new List<double[]>(50); // 连续50个周期的Tcp坐标
        protected List<double[]> continuousFlangePositions = new List<double[]>(50); // 连续50个周期的法兰坐标
        protected List<double[]> continuousOriginalFlangeForces = new List<double[]>(50); // 连续50个周期的原始法兰力信号
        protected List<double[]> continuousZeroedOriginalFlangeForces = new List<double[]>(50); // 连续50个周期的软零化原始法兰力信号
        protected List<double[]> continuousRemoveGravityFlangeForces = new List<double[]>(50); // 连续50个周期的去重力法兰力信号
        protected List<double[]> continuousRemoveGravityTcpForces = new List<double[]>(50); // 连续50个周期的去重力TCP力信号

        protected RobotStatus robotStatus = RobotStatus.PowerOff; // 机械臂状态

        protected double[] flangeToTcp = new double[6]; // 法兰系到工具系的转换关系
        protected double[,] modifiedParameter; // 重力修正因子
        protected ForceModifiedMode modifiedMethod = ForceModifiedMode.NoModified; // 重力修正方式

        protected double maxAngleRange = 0; // 探头标定最大角度范围
        protected byte rollFlag = 0; // 穿刺姿态标号
        protected const double minPitchAngle = -3.0 * Math.PI / 20.0; // 穿刺标定最小俯仰角
        protected const double maxPitchAngle = Math.PI / 3.0; // 穿刺标定最大俯仰角
        #endregion

        #region 字段
        protected CancellationTokenSource listenCancelSource; // 停止监听源
        protected Task listenFromURTask; // UR30003端口监听任务

        protected RobotProgramType robotProgramVersion = RobotProgramType.SW34; // 机器人程序类型
        private int recievedByteLength = 1060; // 接收到的字节长度
        protected int continuousNumOfInvalidRecievedDatas = 0; // 连续收到错误的状态字节数

        protected const int intervalForSendParamsToUI = 15; // 间隔多少周期发送一遍实时数据以供UI显示
        protected int counterForSendParamsToUI = 1; // 实时数据发送间隔周期计数器

        protected bool currentSafetyEnable = false; // 电流安全使能
        protected double[] currentSafetyMaxDifference = new double[6]; // 关节电流最大容差
        protected const int intervalForCurrentSafety = 5; // 采集到多少个连续周期的过流数据才判断为过流
        protected int counterForCurrentSafety = 0; // 过流个数计数器
        protected bool currentOverflow = false; // 是否过流

        protected bool forceSafetyEnable = false; // 力安全使能
        protected double forceSafetyMaxMagnitude = 0; // 力最大安全值
        protected double torqueSafetyMaxMagnitude = 0; // 力矩最大安全值
        protected const int intervalForForceSafety = 5; // 采集到多少个连续周期的力超限数据才判断为力超限
        protected int counterForForceSafety = 0; // 力超限个数计数器
        protected bool forceOverflow = false; // 是否力超限

        protected bool toolIOEnable = false; // 工具IO使能
        protected const int intervalForIO = 10; // 采集到多少个连续周期的IO打开才判断为该IO输入有效
        protected int[] counterForIO = new int[2]; // IO有效个数计数器
        protected bool[] toolPressed = new bool[2]; // IO是否有效输入

        public delegate void SendDoubleArray(double[] Datas); // double数组发送委托
        public delegate void SendShort(short Datas); // short类型发送委托
        /// <summary>
        /// UI数据发送事件
        /// </summary>
        public event SendDoubleArray OnSendParams;
        /// <summary>
        /// 发送各类紧急消息，包括急停、过流、力超限等等
        /// </summary>
        public event SendShort OnSendEmergencyInformation;

        private static readonly object lockedVariable1 = new object(); // 线程锁变量1 锁TCP安装位置读写
        private static readonly object lockedVariable2 = new object(); // 线程锁变量2 锁电流容差读写
        private static readonly object lockedVariable3 = new object(); // 线程锁变量3 锁力和力矩大小极限读写
        private static readonly object lockedVariable4 = new object(); // 线程锁变量4 锁连续力信号读写
        private static readonly object lockedVariable5 = new object(); // 线程锁变量5 锁当前TCP位置坐标读写
        private static readonly object lockedVariable6 = new object(); // 线程锁变量6 锁当前程序状态读写
        private static readonly object lockedVariable7 = new object(); // 线程锁变量7 锁重力修正因子和方式读写
        private static readonly object lockedVariable8 = new object(); // 线程锁变量8 锁连续TCP力信号读写

        protected bool emergencyStopMessageBlock = false; // 急停消息阻塞器
        protected bool protectiveStopMessageBlock = false; // 保护停止消息阻塞器
        #endregion

        #region 字段 502端口 Modbus
        protected int modbusCount = 0; // Modbus数据包发送计数
        protected const int modbusMaxCount = 50; // 间隔多少周期发送一次Modbus数据包
        protected bool ifKeepSendModbusPackage = true; // 是否持续发送Modbus包
        #endregion

        #region 属性
        /// <summary>
        /// 电流安全使能
        /// </summary>
        public bool CurrentSafetyEnable
        {
            get { return currentSafetyEnable; }
            set { currentSafetyEnable = value; }
        }

        /// <summary>
        /// 电流安全最大容差，只能设置
        /// </summary>
        public double[] CurrentSafetyMaxDifference
        {
            set
            {
                lock (lockedVariable2)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        currentSafetyMaxDifference[i] = value[i];
                    }
                }
            }
        }

        /// <summary>
        /// 电流是否超限
        /// </summary>
        public bool CurrentOverflow
        {
            get { return currentOverflow; }
            set { currentOverflow = value; }
        }

        /// <summary>
        /// 力安全使能
        /// </summary>
        public bool ForceSafetyEnable
        {
            get { return forceSafetyEnable; }
            set { forceSafetyEnable = value; }
        }

        /// <summary>
        /// 力和力矩的最大安全值，只能设置
        /// </summary>
        public double[] ForceSafetyMaxMagnitude
        {
            set
            {
                lock (lockedVariable3)
                {
                    forceSafetyMaxMagnitude = value[0];
                    torqueSafetyMaxMagnitude = value[1];
                }
            }
        }

        /// <summary>
        /// 力和力矩是否超限
        /// </summary>
        public bool ForceOverflow
        {
            get { return forceOverflow; }
            set { forceOverflow = value; }
        }

        /// <summary>
        /// 工具IO使能
        /// </summary>
        public bool ToolIOEnable
        {
            get { return toolIOEnable; }
            set { toolIOEnable = value; }
        }

        /// <summary>
        /// 法兰系到工具系的转换关系，只能设置
        /// </summary>
        public double[] FlangeToTcp
        {
            set
            {
                lock (lockedVariable1)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        flangeToTcp[i] = value[i];
                    }
                }
            }
        }

        /// <summary>
        /// 重力修正因子，只能设置
        /// </summary>
        public double[,] ModifiedParameter
        {
            set
            {
                lock (lockedVariable7)
                {
                    modifiedParameter = (double[,])value.Clone();
                }
            }
        }

        /// <summary>
        /// 重力修正方式，只能设置
        /// </summary>
        public ForceModifiedMode ModifiedMethod
        {
            set
            {
                lock (lockedVariable7)
                {
                    modifiedMethod = value;
                }
            }
        }

        /// <summary>
        /// 连续50个周期的原始法兰力信号
        /// </summary>
        protected double[][] ContinuousOriginalFlangeForces
        {
            get
            {
                lock (lockedVariable4)
                {
                    return continuousOriginalFlangeForces.ToArray();
                }
            }
        }

        /// <summary>
        /// 连续50个周期的TCP力信号
        /// </summary>
        protected double[][] ContinuousRemoveGravityTcpForces
        {
            get
            {
                lock (lockedVariable8)
                {
                    return continuousRemoveGravityTcpForces.ToArray();
                }
            }
        }

        /// <summary>
        /// 当前TCP坐标
        /// </summary>
        public double[] PositionsTcpActual
        {
            get
            {
                lock (lockedVariable5)
                {
                    return (double[])positionsTcpActual.Clone();
                }
            }
        }

        /// <summary>
        /// 当前程序状态
        /// </summary>
        public double ProgramState
        {
            get
            {
                lock (lockedVariable6)
                {
                    return programState;
                }
            }
        }

        /// <summary>
        /// 穿刺姿态标号，只能设置
        /// </summary>
        public byte RollFlag
        {
            get { return rollFlag; }
            set { rollFlag = value; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="TcpCartesianPoint">TCP安装坐标</param>
        /// <param name="CurrentProtect">是否启动电流保护</param>
        /// <param name="ForceProtect">是否启动力超限保护</param>
        /// <param name="ToolIOAllow">是否使能工具IO输入</param>
        /// <param name="CurrentProtectLimits">电流保护极限</param>
        /// <param name="ForceProtectLimits">力保护极限</param>
        /// <param name="ProgramType">机器人程序类型</param>
        public UR30003Connector(double[] TcpCartesianPoint, bool CurrentProtect, bool ForceProtect, bool ToolIOAllow, double[] CurrentProtectLimits, double[] ForceProtectLimits, RobotProgramType ProgramType)
        {
            flangeToTcp = (double[])TcpCartesianPoint.Clone();

            currentSafetyEnable = CurrentProtect;
            forceSafetyEnable = ForceProtect;
            toolIOEnable = ToolIOAllow;

            currentSafetyMaxDifference[0] = CurrentProtectLimits[0];
            currentSafetyMaxDifference[1] = CurrentProtectLimits[1];
            currentSafetyMaxDifference[2] = CurrentProtectLimits[2];
            currentSafetyMaxDifference[3] = CurrentProtectLimits[3];
            currentSafetyMaxDifference[4] = CurrentProtectLimits[4];
            currentSafetyMaxDifference[5] = CurrentProtectLimits[5];

            forceSafetyMaxMagnitude = ForceProtectLimits[0];
            torqueSafetyMaxMagnitude = ForceProtectLimits[1];

            robotProgramVersion = ProgramType;
            switch (robotProgramVersion)
            {
                case RobotProgramType.SW34:
                    recievedByteLength = 1060;
                    break;
                case RobotProgramType.SW35:
                    recievedByteLength = 1108;
                    break;
                default:
                    recievedByteLength = 1060;
                    break;
            }

            for (int i = 0; i < 50; i++)
            {
                continuousTcpPositions.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
                continuousFlangePositions.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
                continuousOriginalFlangeForces.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
                continuousZeroedOriginalFlangeForces.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
                continuousRemoveGravityFlangeForces.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
                continuousRemoveGravityTcpForces.Add(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            }

            listenCancelSource = new CancellationTokenSource();
            listenFromURTask = new Task(() => ListenFromURFunction(listenCancelSource.Token));
        }

        /// <summary>
        /// 创建30003端口通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="IfLoose">是否放松起始超时时间</param>
        /// <param name="Port">远程端口号，默认30003</param>
        public void Creat30003Client(string IP, int TimeOut, bool IfLoose, int Port = 30003)
        {
            CreatClient(IP, Port, TimeOut, IfLoose);
        }

        /// <summary>
        /// 创建监听30003端口的新任务，并开始监听
        /// </summary>
        public void CreatListenFromURTask()
        {
            if (listenFromURTask.Status.Equals(TaskStatus.Created))
            {
                listenFromURTask.Start();
                return;
            }
            else if (listenFromURTask.IsCompleted)
            {
                listenCancelSource = new CancellationTokenSource();
                listenFromURTask = new Task(() => ListenFromURFunction(listenCancelSource.Token));
                listenFromURTask.Start();
                return;
            }

            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Try to create UR realdata listener when another listener has not been released yet, so do nothing now.");
        }

        /// <summary>
        /// 停止监听30003端口，阻塞到线程结束
        /// </summary>
        public void StopListenFromURThread()
        {
            if (listenFromURTask.Status.Equals(TaskStatus.Running))
            {
                listenCancelSource.Cancel();

                listenFromURTask.Wait();
                return;
            }

            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Try to stop UR realdata listener when the listener is not running now, so do nothing.");
        }

        /// <summary>
        /// 30003端口监听执行的操作
        /// </summary>
        protected void ListenFromURFunction(CancellationToken CancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "A realdata listener is going to run.");

            while (true)
            {
                if (CancelFlag.IsCancellationRequested)
                {
                    break;
                }

                bool recievedCorrect = UnpackRecievedDatas();
                CopyForceData();

                if (recievedCorrect)
                {
                    BaseWorkAfterRecievedCorrectDatas();
                    WorkBasedOnListeningAsTimer();
                }

                if ((++counterForSendParamsToUI) > intervalForSendParamsToUI)
                {
                    counterForSendParamsToUI = 1;
                    OnSendParams(CopyAndSendPartOfParams());
                }
            }

            URConnectionBroken();
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "A realdata listener is going to stop.");
        }

        /// <summary>
        /// 接收实时控制状态并解析，必须重载
        /// </summary>
        /// <returns>本次接收是否成功</returns>
        protected virtual bool UnpackRecievedDatas()
        {
            byte[] getDatas = WaitForFeedback(recievedByteLength);

            if (getDatas.Length < 1) // 收到的字节长度错误
            {
                continuousNumOfInvalidRecievedDatas++;
                if (continuousNumOfInvalidRecievedDatas > 5)
                {
                    continuousNumOfInvalidRecievedDatas = 0;
                    throw new Exception("invalid recieved datas.");
                }
                return false;
            }
            else
            {
                int recLength = BitConverter.ToInt32(
                                           BitConverter.GetBytes(
                                           IPAddress.NetworkToHostOrder(
                                           BitConverter.ToInt32(getDatas, (int)RealTimeDatasMarks.Size))), 0);
                if (recLength != recievedByteLength) // 收到的字节顺序错误
                {
                    continuousNumOfInvalidRecievedDatas++;
                    if (continuousNumOfInvalidRecievedDatas > 5)
                    {
                        continuousNumOfInvalidRecievedDatas = 0;
                        throw new Exception("invalid recieved datas.");
                    }
                    return false;
                }
                else // 收到的字节正常
                {
                    totalLength = recLength;
                    continuousNumOfInvalidRecievedDatas = 0;
                }
            }

            // 解析收到的字节流
            double time = BitConverter.Int64BitsToDouble(
                                            IPAddress.NetworkToHostOrder(
                                            BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Time)));
            robotTimeMicroSecond = (int)Math.Round((time - Math.Floor(time)) * 1000);
            double roundTime = Math.Round(time);
            robotTimeHour = (int)Math.Floor(roundTime / 3600.0);
            robotTimeMinute = (int)Math.Floor((roundTime - robotTimeHour * 3600.0) / 60.0);
            robotTimeSecond = (int)(roundTime - robotTimeHour * 3600.0 - robotTimeMinute * 60.0);

            for (int k = 0; k < 6; k++)
            {
                positionsJointTarget[k] = BitConverter.Int64BitsToDouble(
                                                                      IPAddress.NetworkToHostOrder(
                                                                      BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Targetq + k * 8)));

                speedsJointTarget[k] = BitConverter.Int64BitsToDouble(
                                                                   IPAddress.NetworkToHostOrder(
                                                                   BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Targetqd + k * 8)));

                accelerationsJointTarget[k] = BitConverter.Int64BitsToDouble(
                                                                              IPAddress.NetworkToHostOrder(
                                                                              BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Targetqdd + k * 8)));

                currentsJointTarget[k] = BitConverter.Int64BitsToDouble(
                                                                      IPAddress.NetworkToHostOrder(
                                                                      BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.TargetI + k * 8)));

                torquesJointTarget[k] = BitConverter.Int64BitsToDouble(
                                                                     IPAddress.NetworkToHostOrder(
                                                                     BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.TargetT + k * 8)));

                positionsJointActual[k] = BitConverter.Int64BitsToDouble(
                                                                      IPAddress.NetworkToHostOrder(
                                                                      BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Actualq + k * 8)));

                speedsJointActual[k] = BitConverter.Int64BitsToDouble(
                                                                   IPAddress.NetworkToHostOrder(
                                                                   BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Actualqd + k * 8)));

                currentsJointActual[k] = BitConverter.Int64BitsToDouble(
                                                                      IPAddress.NetworkToHostOrder(
                                                                      BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ActualI + k * 8)));

                currentsJointControl[k] = BitConverter.Int64BitsToDouble(
                                                                        IPAddress.NetworkToHostOrder(
                                                                        BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ControlI + k * 8)));

                lock (lockedVariable5)
                {
                    positionsTcpActual[k] = BitConverter.Int64BitsToDouble(
                                                                                         IPAddress.NetworkToHostOrder(
                                                                                         BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Actualp + k * 8)));
                }

                speedsTcpActual[k] = BitConverter.Int64BitsToDouble(
                                                                   IPAddress.NetworkToHostOrder(
                                                                   BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Actualpd + k * 8)));

                forcesTcpActual[k] = BitConverter.Int64BitsToDouble(
                                                                  IPAddress.NetworkToHostOrder(
                                                                  BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.TCPF + k * 8)));

                positionsTcpTarget[k] = BitConverter.Int64BitsToDouble(
                                                                     IPAddress.NetworkToHostOrder(
                                                                     BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Targetp + k * 8)));

                speedsTcpTarget[k] = BitConverter.Int64BitsToDouble(
                                                                  IPAddress.NetworkToHostOrder(
                                                                  BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Targetpd + k * 8)));

                temperaturesJoint[k] = BitConverter.Int64BitsToDouble(
                                                                  IPAddress.NetworkToHostOrder(
                                                                  BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.Temperature + k * 8)));

                voltagesJoint[k] = BitConverter.Int64BitsToDouble(
                                                         IPAddress.NetworkToHostOrder(
                                                         BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.VoltageJ + k * 8)));
            }

            int input = Convert.ToInt32(
                              BitConverter.Int64BitsToDouble(
                              IPAddress.NetworkToHostOrder(
                              BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.DigitalIn))));
            inputDigital[0] = ((input & 1) == 1);
            inputDigital[1] = ((input & 2) == 2);
            inputDigital[2] = ((input & 4) == 4);
            inputDigital[3] = ((input & 8) == 8);
            inputDigital[4] = ((input & 16) == 16);
            inputDigital[5] = ((input & 32) == 32);
            inputDigital[6] = ((input & 64) == 64);
            inputDigital[7] = ((input & 128) == 128);

            inputConfig[0] = ((input & 256) == 256);
            inputConfig[1] = ((input & 512) == 512);
            inputConfig[2] = ((input & 1024) == 1024);
            inputConfig[3] = ((input & 2048) == 2048);
            inputConfig[4] = ((input & 4096) == 4096);
            inputConfig[5] = ((input & 8192) == 8192);
            inputConfig[6] = ((input & 16384) == 16384);
            inputConfig[7] = ((input & 32768) == 32768);

            inputTool[0] = ((input & 65536) == 65536);
            inputTool[1] = ((input & 131072) == 131072);

            controllerTimer = BitConverter.Int64BitsToDouble(
                                            IPAddress.NetworkToHostOrder(
                                            BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ControlT)));

            double moder = BitConverter.Int64BitsToDouble(
                                       IPAddress.NetworkToHostOrder(
                                       BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ModeR)));
            modeRobotDisconnected = (moder == 0);
            modeRobotConfirmSafety = (moder == 1);
            modeRobotBooting = (moder == 2);
            modeRobotPowerOff = (moder == 3);
            modeRobotPowerOn = (moder == 4);
            modeRobotIdle = (moder == 5);
            modeRobotBackDrive = (moder == 6);
            modeRobotRunning = (moder == 7);
            modeRobotUpdatingFirmware = (moder == 8);

            double[] modej = new double[6];
            for (int countnum = 0; countnum < 6; countnum++)
            {
                modej[countnum] = BitConverter.Int64BitsToDouble(
                                                  IPAddress.NetworkToHostOrder(
                                                  BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ModeJ + countnum * 8)));
                modeJointsShuttingdown[countnum] = (modej[countnum] == 236);
                modeJointsPartDCalibration[countnum] = (modej[countnum] == 237);
                modeJointsBackDrive[countnum] = (modej[countnum] == 238);
                modeJointsPowerOff[countnum] = (modej[countnum] == 239);
                modeJointsNotResponding[countnum] = (modej[countnum] == 245);
                modeJointsMotorInitialisation[countnum] = (modej[countnum] == 246);
                modeJointsBooting[countnum] = (modej[countnum] == 247);
                modeJointsPartDCalibrationError[countnum] = (modej[countnum] == 248);
                modeJointsBootLoader[countnum] = (modej[countnum] == 249);
                modeJointsCalibration[countnum] = (modej[countnum] == 250);
                modeJointsFault[countnum] = (modej[countnum] == 252);
                modeJointsRunning[countnum] = (modej[countnum] == 253);
                modeJointsIdle[countnum] = (modej[countnum] == 255);
            }

            double modes = BitConverter.Int64BitsToDouble(
                                        IPAddress.NetworkToHostOrder(
                                        BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ModeS)));
            modeSafetyNormal = (modes == 1);
            modeSafetyReduced = (modes == 2);
            modeSafetyProtectiveStop = (modes == 3);
            modeSafetyRecovery = (modes == 4);
            modeSafetySafeGuardStop = (modes == 5);
            modeSafetySystemEmergencyStop = (modes == 6);
            modeSafetyRobotEmergencyStop = (modes == 7);
            modeSafetyViolation = (modes == 8);
            modeSafetyFault = (modes == 9);

            for (int k = 0; k < 3; k++)
            {
                accelerationsTcpActual[k] = BitConverter.Int64BitsToDouble(
                                                                              IPAddress.NetworkToHostOrder(
                                                                              BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ToolAcceleration + k * 8)));
            }

            speedScaling = BitConverter.Int64BitsToDouble(
                                        IPAddress.NetworkToHostOrder(
                                        BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.SpeedScaling)));

            momentumTcpActual = BitConverter.ToDouble(
                                                                  BitConverter.GetBytes(
                                                                  IPAddress.NetworkToHostOrder(
                                                                  BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.TorqueNorm))), 0);

            voltageMain = BitConverter.ToDouble(
                                               BitConverter.GetBytes(
                                               IPAddress.NetworkToHostOrder(
                                               BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.VoltageMain))), 0);

            voltageRobot = BitConverter.ToDouble(
                                                 BitConverter.GetBytes(
                                                 IPAddress.NetworkToHostOrder(
                                                 BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.VoltageRobot))), 0);

            currentRobot = BitConverter.ToDouble(
                                                 BitConverter.GetBytes(
                                                 IPAddress.NetworkToHostOrder(
                                                 BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.CurrentRobot))), 0);

            int output = Convert.ToInt32(
                                 BitConverter.Int64BitsToDouble(
                                 IPAddress.NetworkToHostOrder(
                                 BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.DigitalOut))));
            outputDigital[0] = ((output & 1) == 1);
            outputDigital[1] = ((output & 2) == 2);
            outputDigital[2] = ((output & 4) == 4);
            outputDigital[3] = ((output & 8) == 8);
            outputDigital[4] = ((output & 16) == 16);
            outputDigital[5] = ((output & 32) == 32);
            outputDigital[6] = ((output & 64) == 64);
            outputDigital[7] = ((output & 128) == 128);

            outputConfig[0] = ((output & 256) == 256);
            outputConfig[1] = ((output & 512) == 512);
            outputConfig[2] = ((output & 1024) == 1024);
            outputConfig[3] = ((output & 2048) == 2048);
            outputConfig[4] = ((output & 4096) == 4096);
            outputConfig[5] = ((output & 8192) == 8192);
            outputConfig[6] = ((output & 16384) == 16384);
            outputConfig[7] = ((output & 32768) == 32768);

            outputTool[0] = ((output & 65536) == 65536);
            outputTool[1] = ((output & 131072) == 131072);

            lock (lockedVariable6)
            {
                programState = BitConverter.Int64BitsToDouble(
                                          IPAddress.NetworkToHostOrder(
                                          BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.StateProg)));
            }

            if (robotProgramVersion == RobotProgramType.SW35)
            {
                for (int k = 0; k < 3; k++)
                {
                    elbowPosition[k] = BitConverter.Int64BitsToDouble(
                                                   IPAddress.NetworkToHostOrder(
                                                   BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ElbowPos + k * 8)));

                    elbowVelocity[k] = BitConverter.Int64BitsToDouble(
                                                   IPAddress.NetworkToHostOrder(
                                                   BitConverter.ToInt64(getDatas, (int)RealTimeDatasMarks.ElbowV + k * 8)));
                }
            }

            return true;
        }

        /// <summary>
        /// 拷贝力信号数据，必须重载
        /// </summary>
        protected virtual void CopyForceData() { }

        /// <summary>
        /// 收到准确的UR实时数据后需要做的工作
        /// 0. 获得工具在法兰系中的坐标
        /// 1. 进行坐标变换，获得法兰在基系中的坐标
        /// 2. 根据当前位姿，计算去重力后的力参数
        /// 3. 进行坐标变换，获得基系和Tcp系下的去重力后的力参数
        /// 4. 维护多时刻数据
        /// 5. 状态检查，更新机械臂状态
        /// 6. 状态检查，检查反馈的运行情况
        /// 7. 状态检查，检查电流安全限制
        /// 8. 状态检查，检查力安全限制
        /// 9. 状态检查，检查IO状态
        /// </summary>
        protected virtual void BaseWorkAfterRecievedCorrectDatas()
        {
            // 0. 获得工具在法兰系中的坐标
            double[] tcpAtFlange = new double[6];
            lock (lockedVariable1)
            {
                for (int i = 0; i < 6; i++)
                {
                    tcpAtFlange[i] = flangeToTcp[i];
                }
            }

            // 1. 进行坐标变换，获得法兰在基系中的坐标
            positionsFlangeActual = URMath.FindThirdReferenceToFirstReference(positionsTcpActual, URMath.ReverseReferenceRelationship(tcpAtFlange));

            // 2. 根据当前位姿，计算去重力后的力参数 
            lock (lockedVariable7)
            {
                removeGravityFlangeForces = CalculateForceWithoutGravity(positionsTcpActual, positionsFlangeActual, zeroedOriginalFlangeForces, modifiedParameter, modifiedMethod);
            }

            // 3. 进行坐标变换，获得基系和Tcp系下的去重力后的力参数
            double flangeForceMagnitude = URMath.LengthOfArray(new double[] { removeGravityFlangeForces[0], removeGravityFlangeForces[1], removeGravityFlangeForces[2] });
            double[] flangeForceDirection;
            if (flangeForceMagnitude == 0.0)
            {
                flangeForceDirection = new double[3] { 0.0, 0.0, 0.0 };
            }
            else
            {
                flangeForceDirection = new double[3] { removeGravityFlangeForces[0] / flangeForceMagnitude, removeGravityFlangeForces[1] / flangeForceMagnitude, removeGravityFlangeForces[2] / flangeForceMagnitude };
            }
            double flangeTorqueMagnitude = URMath.LengthOfArray(new double[] { removeGravityFlangeForces[3], removeGravityFlangeForces[4], removeGravityFlangeForces[5] });
            double[] flangeTorqueDirection;
            if (flangeTorqueMagnitude == 0)
            {
                flangeTorqueDirection = new double[3] { 0.0, 0.0, 0.0 };
            }
            else
            {
                flangeTorqueDirection = new double[3] { removeGravityFlangeForces[3] / flangeTorqueMagnitude, removeGravityFlangeForces[4] / flangeTorqueMagnitude, removeGravityFlangeForces[5] / flangeTorqueMagnitude };
            }
            removeGravityBaseForces = URMath.FindCordinateToSecondReferenceFromFirstReference(new double[] { flangeForceMagnitude, flangeForceDirection[0], flangeForceDirection[1], flangeForceDirection[2], flangeTorqueMagnitude, flangeTorqueDirection[0], flangeTorqueDirection[1], flangeTorqueDirection[2] },
                                                        URMath.ReverseReferenceRelationship(positionsFlangeActual));
            removeGravityTcpForces = URMath.FindCordinateToSecondReferenceFromFirstReference(new double[] { flangeForceMagnitude, flangeForceDirection[0], flangeForceDirection[1], flangeForceDirection[2], flangeTorqueMagnitude, flangeTorqueDirection[0], flangeTorqueDirection[1], flangeTorqueDirection[2] },
                                                       tcpAtFlange);

            // 4. 维护多时刻数据
            continuousTcpPositions.RemoveAt(0);
            continuousTcpPositions.Add((double[])positionsTcpActual.Clone());
            continuousFlangePositions.RemoveAt(0);
            continuousFlangePositions.Add((double[])positionsFlangeActual.Clone());
            lock (lockedVariable4)
            {
                continuousOriginalFlangeForces.RemoveAt(0);
                continuousOriginalFlangeForces.Add((double[])originalFlangeForces.Clone());
            }
            continuousZeroedOriginalFlangeForces.RemoveAt(0);
            continuousZeroedOriginalFlangeForces.Add((double[])zeroedOriginalFlangeForces.Clone());
            continuousRemoveGravityFlangeForces.RemoveAt(0);
            continuousRemoveGravityFlangeForces.Add((double[])removeGravityFlangeForces.Clone());
            lock (lockedVariable8)
            {
                continuousRemoveGravityTcpForces.RemoveAt(0);
                continuousRemoveGravityTcpForces.Add((double[])removeGravityTcpForces.Clone());
            }

            // 5. 状态检查，更新机械臂状态
            if (modeRobotPowerOff)
            {
                robotStatus = RobotStatus.PowerOff;
            }
            else if (modeRobotRunning)
            {
                robotStatus = RobotStatus.Running;
            }
            else if (modeRobotIdle)
            {
                robotStatus = RobotStatus.Idle;
            }
            else if (modeRobotPowerOn)
            {
                robotStatus = RobotStatus.PowerOn;
            }
            else
            {
                robotStatus = RobotStatus.PowerOff;
            }

            // 6. 状态检查，检查反馈的运行情况
            if (modeSafetyRobotEmergencyStop || modeSafetySystemEmergencyStop)
            {
                if (!emergencyStopMessageBlock)
                {
                    OnSendEmergencyInformation((short)RobotEmergency.EmergencyStop);
                    Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Robot has emergency stopped.");

                    emergencyStopMessageBlock = true;
                }
            }
            else if (modeSafetyProtectiveStop)
            {
                if (!protectiveStopMessageBlock)
                {
                    OnSendEmergencyInformation((short)RobotEmergency.ProtectiveStop);
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Robot has protective stopped.");

                    protectiveStopMessageBlock = true;
                }
            }
            else
            {
                emergencyStopMessageBlock = false;
                protectiveStopMessageBlock = false;
            }

            // 7. 状态检查，检查电流安全限制
            if (currentSafetyEnable)
            {
                if (modeRobotRunning && !currentOverflow && IfCurrentOverflow())
                {
                    if (counterForCurrentSafety >= intervalForCurrentSafety)
                    {
                        currentOverflow = true;
                        counterForCurrentSafety = 0;
                        DealWithEmergency(RobotEmergency.CurrentOverflow);
                        OnSendEmergencyInformation((short)RobotEmergency.CurrentOverflow);
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Robot joint current has overflowed.");
                    }
                    else
                    {
                        counterForCurrentSafety++;
                    }
                }
                else
                {
                    counterForCurrentSafety = 0;
                }
            }
            else
            {
                counterForCurrentSafety = 0;
                currentOverflow = false;
            }

            // 8. 状态检查，检查力安全限制
            if (forceSafetyEnable)
            {
                if (modeRobotRunning && !forceOverflow && IfForceOverflow())
                {
                    if (counterForForceSafety >= intervalForForceSafety)
                    {
                        forceOverflow = true;
                        counterForForceSafety = 0;
                        DealWithEmergency(RobotEmergency.ForceOverflow);
                        OnSendEmergencyInformation((short)RobotEmergency.ForceOverflow);
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Tool force has overflowed.");
                    }
                    else
                    {
                        counterForForceSafety++;
                    }
                }
                else
                {
                    counterForForceSafety = 0;
                }
            }
            else
            {
                counterForForceSafety = 0;
                forceOverflow = false;
            }

            // 9. 状态检查，检查IO状态
            if (toolIOEnable)
            {
                if (inputTool[0])
                {
                    if (counterForIO[0] >= intervalForIO && !toolPressed[0])
                    {
                        toolPressed[0] = true;
                        counterForIO[0] = 0;
                        DealWithIOInput(0);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Tool input IO {0} has pressed.");
                    }
                    else
                    {
                        counterForIO[0]++;
                    }
                }
                else
                {
                    toolPressed[0] = false;
                    counterForIO[0] = 0;
                }

                if (inputTool[1])
                {
                    if (counterForIO[1] >= intervalForIO && !toolPressed[1])
                    {
                        toolPressed[1] = true;
                        counterForIO[1] = 0;
                        DealWithIOInput(1);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Tool input IO {1} has pressed.");
                    }
                    else
                    {
                        counterForIO[1]++;
                    }
                }
                else
                {
                    toolPressed[1] = false;
                    counterForIO[1] = 0;
                }
            }
            else
            {
                counterForIO[0] = 0;
                toolPressed[0] = false;
                counterForIO[1] = 0;
                toolPressed[1] = false;
            }
        }

        /// <summary>
        /// 判断本周期是否过流
        /// </summary>
        /// <returns>返回判断结果</returns>
        protected bool IfCurrentOverflow()
        {
            double[] currentMaxLimits = new double[6];
            lock (lockedVariable2)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentMaxLimits[i] = currentSafetyMaxDifference[i];
                }
            }

            if (Math.Abs(currentsJointActual[0] - currentsJointTarget[0]) > currentMaxLimits[0] ||
                Math.Abs(currentsJointActual[1] - currentsJointTarget[1]) > currentMaxLimits[1] ||
                Math.Abs(currentsJointActual[2] - currentsJointTarget[2]) > currentMaxLimits[2] ||
                Math.Abs(currentsJointActual[3] - currentsJointTarget[3]) > currentMaxLimits[3] ||
                Math.Abs(currentsJointActual[4] - currentsJointTarget[4]) > currentMaxLimits[4] ||
                Math.Abs(currentsJointActual[5] - currentsJointTarget[5]) > currentMaxLimits[5])
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// 判断本周期是否力超限
        /// </summary>
        /// <returns>返回判断结果</returns>
        protected bool IfForceOverflow()
        {
            double forceLimit = 0, torqueLimit = 0;
            lock (lockedVariable3)
            {
                forceLimit = forceSafetyMaxMagnitude;
                torqueLimit = torqueSafetyMaxMagnitude;
            }

            double forceMagnitude = URMath.LengthOfArray(new double[] { removeGravityFlangeForces[0], removeGravityFlangeForces[1], removeGravityFlangeForces[2] });
            double torqueMagnitude = URMath.LengthOfArray(new double[] { removeGravityFlangeForces[3], removeGravityFlangeForces[4], removeGravityFlangeForces[5] });
            if (forceMagnitude > forceLimit || torqueMagnitude > torqueLimit)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// 紧急事件处理，必须重载
        /// </summary>
        /// <param name="EmergencyState">紧急状态</param>
        protected virtual void DealWithEmergency(RobotEmergency EmergencyState) { }

        /// <summary>
        /// 处理工具IO输入响应事件，需要时重载
        /// </summary>
        /// <param name="IONumber">工具IO号</param>
        protected virtual void DealWithIOInput(int IONumber) { }

        /// <summary>
        /// 探头去重力，必须重载
        /// </summary>
        /// <param name="ToolPosition">工具位姿</param>
        /// <param name="FlangePosition">法兰位姿</param>
        /// <param name="OriginalForce">原始力信息</param>
        /// <param name="ModifiedParameter">修正因子</param>
        /// <param name="ModifiedMethod">修正方式</param>
        /// <returns>去重力后的力信息</returns>
        protected virtual double[] CalculateForceWithoutGravity(double[] ToolPosition, double[] FlangePosition, double[] OriginalForce, double[,] ModifiedParameter, ForceModifiedMode ModifiedMethod) { return null; }

        /// <summary>
        /// 利用UR反馈数据的频率作为定时器执行一定的功能，必须重载
        /// </summary>
        protected virtual void WorkBasedOnListeningAsTimer() { }

        /// <summary>
        /// 将需要的参数传给UI
        /// </summary>
        /// <returns>返回要传给UI的参数</returns>
        protected virtual double[] CopyAndSendPartOfParams()
        {
            List<double> transformedList = new List<double>(33);

            transformedList.AddRange(positionsTcpActual);
            transformedList.AddRange(positionsJointActual);
            transformedList.AddRange(temperaturesJoint);
            //transformedList.AddRange(speedsTcpActual);
            transformedList.AddRange(currentsJointActual);

            foreach (bool io in inputTool)
            {
                transformedList.Add((double)(io ? 1.0 : 0.0));
            }

            transformedList.Add((double)robotStatus);
            transformedList.Add(programState);

            transformedList.AddRange(removeGravityTcpForces);
            //transformedList.AddRange(originalFlangeForces);

            return transformedList.ToArray();
        }

        /// <summary>
        /// UR通讯连接中断，必须重载
        /// </summary>
        protected virtual void URConnectionBroken() { }

        /// <summary>
        /// 获得给定Tcp坐标的X轴方向在基系的表示
        /// </summary>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回其X轴方向在基系的表示</returns>
        public double[] XDirectionOfTcpAtBaseReference(double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] tcpToBase = URMath.ReverseReferenceRelationship(TcpCoordinate);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });

            return URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 1.0, 0.0, 0.0 }, qTcpToBase);
        }

        /// <summary>
        /// 获得给定Tcp坐标的Y轴方向在基系的表示
        /// </summary>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回其Y轴方向在基系的表示</returns>
        public double[] YDirectionOfTcpAtBaseReference(double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] tcpToBase = URMath.ReverseReferenceRelationship(TcpCoordinate);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });

            return URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 1.0, 0.0 }, qTcpToBase);
        }

        /// <summary>
        /// 获得给定Tcp坐标的Z轴方向在基系的表示
        /// </summary>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回其Z轴方向在基系的表示</returns>
        public double[] ZDirectionOfTcpAtBaseReference(double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] tcpToBase = URMath.ReverseReferenceRelationship(TcpCoordinate);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });

            return URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, 1.0 }, qTcpToBase);
        }

        /// <summary>
        /// 绕Tcp的X轴旋转一定角度
        /// </summary>
        /// <param name="Angle">旋转的角度</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回旋转后的Tcp坐标</returns>
        public double[] RotateByTcpXAxis(double Angle, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] position = { TcpCoordinate[0], TcpCoordinate[1], TcpCoordinate[2] };
            double[] posture = { TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };

            double[] direction = XDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Angle * direction[0], Angle * direction[1], Angle * direction[2] };

            Quatnum qNextPosture = URMath.QuatnumRotate(new Quatnum[]{
                                                                                                     URMath.AxisAngle2Quatnum(posture),
                                                                                                     URMath.AxisAngle2Quatnum(valueDirection)});
            double[] nextPosture = URMath.Quatnum2AxisAngle(qNextPosture);

            return new double[] { position[0], position[1], position[2], 
                                               nextPosture[0], nextPosture[1], nextPosture[2] };
        }

        /// <summary>
        /// 绕Tcp的Y轴旋转一定角度
        /// </summary>
        /// <param name="Angle">旋转的角度</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回旋转后的Tcp坐标</returns>
        public double[] RotateByTcpYAxis(double Angle, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] position = { TcpCoordinate[0], TcpCoordinate[1], TcpCoordinate[2] };
            double[] posture = { TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };

            double[] direction = YDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Angle * direction[0], Angle * direction[1], Angle * direction[2] };

            Quatnum qNextPosture = URMath.QuatnumRotate(new Quatnum[]{
                                                                                                     URMath.AxisAngle2Quatnum(posture),
                                                                                                     URMath.AxisAngle2Quatnum(valueDirection)});
            double[] nextPosture = URMath.Quatnum2AxisAngle(qNextPosture);

            return new double[] { position[0], position[1], position[2], 
                                               nextPosture[0], nextPosture[1], nextPosture[2] };
        }

        /// <summary>
        /// 绕Tcp的Z轴旋转一定角度
        /// </summary>
        /// <param name="Angle">旋转的角度</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回旋转后的Tcp坐标</returns>
        public double[] RotateByTcpZAxis(double Angle, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] position = { TcpCoordinate[0], TcpCoordinate[1], TcpCoordinate[2] };
            double[] posture = { TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };

            double[] direction = ZDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Angle * direction[0], Angle * direction[1], Angle * direction[2] };

            Quatnum qNextPosture = URMath.QuatnumRotate(new Quatnum[]{
                                                                                                     URMath.AxisAngle2Quatnum(posture),
                                                                                                     URMath.AxisAngle2Quatnum(valueDirection)});
            double[] nextPosture = URMath.Quatnum2AxisAngle(qNextPosture);

            return new double[] { position[0], position[1], position[2], 
                                               nextPosture[0], nextPosture[1], nextPosture[2] };
        }

        /// <summary>
        /// 沿Tcp的X轴前进一定距离
        /// </summary>
        /// <param name="Distance">移动的距离</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回移动后的Tcp坐标</returns>
        public double[] MoveAlongTcpXAxis(double Distance, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] direction = XDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Distance * direction[0], Distance * direction[1], Distance * direction[2] };

            return new double[] { TcpCoordinate[0] + valueDirection[0], TcpCoordinate[1] + valueDirection[1], TcpCoordinate[2] + valueDirection[2], 
                                               TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };
        }

        /// <summary>
        /// 沿Tcp的Y轴前进一定距离
        /// </summary>
        /// <param name="Distance">移动的距离</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回移动后的Tcp坐标</returns>
        public double[] MoveAlongTcpYAxis(double Distance, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] direction = YDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Distance * direction[0], Distance * direction[1], Distance * direction[2] };

            return new double[] { TcpCoordinate[0] + valueDirection[0], TcpCoordinate[1] + valueDirection[1], TcpCoordinate[2] + valueDirection[2], 
                                               TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };
        }

        /// <summary>
        /// 沿Tcp的Z轴前进一定距离
        /// </summary>
        /// <param name="Distance">移动的距离</param>
        /// <param name="TcpCoordinate">给定的Tcp坐标，默认为当前值</param>
        /// <returns>返回移动后的Tcp坐标</returns>
        public double[] MoveAlongTcpZAxis(double Distance, double[] TcpCoordinate = null)
        {
            if (TcpCoordinate == null)
            {
                TcpCoordinate = PositionsTcpActual;
            }

            double[] direction = ZDirectionOfTcpAtBaseReference(TcpCoordinate);
            double[] valueDirection = { Distance * direction[0], Distance * direction[1], Distance * direction[2] };

            return new double[] { TcpCoordinate[0] + valueDirection[0], TcpCoordinate[1] + valueDirection[1], TcpCoordinate[2] + valueDirection[2], 
                                               TcpCoordinate[3], TcpCoordinate[4], TcpCoordinate[5] };
        }

        /// <summary>
        /// 关闭30003端口通讯
        /// </summary>
        public void Close30003Client()
        {
            CloseClient();
        }
        #endregion

    }
}
