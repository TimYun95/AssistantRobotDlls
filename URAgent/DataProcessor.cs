using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Reflection;

using System.Net.NetworkInformation;
using System.IO;
using URNonServo;
using URServo;
using MathFunction;
using LogPrinter;

namespace URCommunication
{
    /// <summary>
    /// OPTO数据处理类，负责与OPTO的通讯数据交互
    /// </summary>
    public class OPTODataProcessor : OPTO49152Connector
    {
        #region 字段
        bool ifOPTOConnected = false; // 是否连接到了OPTO

        public delegate void SendVoid(); // 无参数发送委托
        public event SendVoid OnSendOPTOConnectedBroken; // 发送OPTO通讯连接断开消息
        #endregion

        #region 属性
        /// <summary>
        /// 是否连接到了OPTO
        /// </summary>
        public bool IfOPTOConnected
        {
            get { return ifOPTOConnected; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="SenorVersion">传感器种类</param>
        /// <param name="FrequencySampling">采用频率，默认250Hz</param>
        /// <param name="FilterSampling">截断频率，默认4，即15Hz</param>
        /// <param name="AmountSampling">要采集的数据个数，默认0，即无穷个</param>
        public OPTODataProcessor(SensorType SenorVersion, int FrequencySampling = 250, int FilterSampling = 4, int AmountSampling = 0)
            : base(SenorVersion, FrequencySampling, FilterSampling, AmountSampling) { }

        /// <summary>
        /// 创建49152端口通讯并连接，已重载
        /// </summary>
        /// <param name="SelfIP">自身IP地址</param>
        /// <param name="OppositeIP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="Port">公共端口号，默认49152</param>
        public override void Creat49152Client(string SelfIP, string OppositeIP, int TimeOut, int Port = 49152)
        {
            base.Creat49152Client(SelfIP, OppositeIP, TimeOut, Port);
            ifOPTOConnected = true;
        }

        /// <summary>
        /// 接收实时力信号并解析，已重载
        /// </summary>
        protected override void UnpackRecievedDatas()
        {
            try
            {
                base.UnpackRecievedDatas();
            }
            catch (Exception ex)
            {
                listenCancelSource.Cancel();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "OPTO listener canceled.", ex);
                return;
            }
        }

        /// <summary>
        /// OPTO通讯连接中断，已重载
        /// </summary>
        protected override void OPTOConnectionBroken()
        {
            // 断开OPTO连接
            try
            {
                EndCollectorSend();
                Close49152Client();
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Opto connection crashed.");
            }
            catch { }

            // 发起OPTO连接断开事件
            OnSendOPTOConnectedBroken();
        }

        /// <summary>
        /// 关闭49152端口通讯，必须重载
        /// </summary>
        public override void Close49152Client()
        {
            ifOPTOConnected = false;
            base.Close49152Client();
        }
        #endregion
    }

    /// <summary>
    /// UR数据处理类，负责所有的通讯数据交互和实时控制逻辑
    /// </summary>
    public class URDataProcessor : UR30003Connector
    {
        #region 枚举
        /// <summary>
        /// 网络连接状态
        /// </summary>
        public enum NetConnection : short
        {
            Broken = 0,
            ActiveBroken,
            Connected
        }

        /// <summary>
        /// 机械臂类型
        /// </summary>
        public enum RobotType : byte
        {
            CBUR3 = 0,
            CBUR5,
            CBUR10,
            EUR3,
            EUR5,
            EUR10
        }
        #endregion

        #region 字段
        protected RobotType robotVersion = RobotType.CBUR3; // 机械臂类型
        protected double shoulderSingularDistance = 0.16; // 肩关节奇异距离

        protected string ipOfUR = null; // UR的IP地址
        protected string ipOfOPTO = null; // OPTO的IP地址
        protected string ipOfLocalForOPTO = null; // 连接OPTO的本机IP地址
        protected int timeOut = 0; // 收发的超时时间
        protected bool ifLoose = false; // 是否放松起始超时时间
        protected int checkingTime = 0; // 检查网络间隔时间
        private static readonly object netCheckLocker = new object(); // 网络定时检查锁

        protected UR29999Connector baseController; // 29999端口 基础功能控制器
        protected UR30002Connector commandSender; // 30002端口 指令发送者
        protected UR30004Connector servoSender; // 30004端口 伺服数据发送者
        protected UR502Connector modbusSender; // 502端口 Modbus数据发送者
        protected OPTODataProcessor forceAgent; // 49152端口 力信号代理

        protected const bool ifUse30004Port = false; // 是否使用30004端口

        protected bool ifUseForceAgent = false; // 是否使用力信号代理
        protected System.Timers.Timer netChecker = new System.Timers.Timer(); // 网络侦测定时器

        protected bool ifURConnected = false; // 是否连接到了UR
        /// <summary>
        /// 发送UR通讯断开或连接消息
        /// </summary>
        public event SendShort OnSendURBrokenOrConnected;

        protected bool ifActiveCloseConnection = false; // 是否主动关闭通讯连接

        protected double[] zeroForceBias = new double[6]; // 力信号零点偏置
        private static readonly object zeroForceLocker = new object(); // 力信号零点偏置锁

        public delegate void SendVoid(); // 无参数发送委托
        public event SendVoid OnSendZeroedForceCompeleted; // 发送力信号软零化完成
        public delegate void SendDoubleMatrix(double[,] Datas); // double二位数组发送委托
        public event SendShort OnSendPreciseCalibrationProcess; // 发送力信号标定过程
        public event SendDoubleMatrix OnSendPreciseCalibrationDatas; // 发送力信号标定数据
        public event SendShort OnSendEntireCalibrationProcess; // 发送完整力信号标定过程
        public event SendDoubleMatrix OnSendEntireCalibrationDatas; // 发送完整力信号标定数据

        protected bool ifOpenSingularCheck = true; // 是否打开奇异点检查
        protected bool ifNearSingularPoint = false; // 是否临近奇异点
        public event SendShort OnSendNearSingularPoint; // 发送临近奇异点消息
        #endregion

        #region 属性
        /// <summary>
        /// 是否使用力信号代理
        /// </summary>
        public bool IfUseForceAgent
        {
            set { ifUseForceAgent = value; }
        }

        /// <summary>
        /// UR的IP地址
        /// </summary>
        public string IPOfUR
        {
            get { return ipOfUR; }
            set { ipOfUR = value; }
        }

        /// <summary>
        /// OPTO的IP地址
        /// </summary>
        public string IPOfOPTO
        {
            get { return ipOfOPTO; }
            set { ipOfOPTO = value; }
        }

        /// <summary>
        /// 连接OPTO的本机IP地址
        /// </summary>
        public string IPOfLocalForOPTO
        {
            get { return ipOfLocalForOPTO; }
            set { ipOfLocalForOPTO = value; }
        }

        /// <summary>
        /// 收发的超时时间
        /// </summary>
        public int TimeOut
        {
            get { return timeOut; }
            set { timeOut = value; }
        }

        /// <summary>
        /// 检查网络间隔时间
        /// </summary>
        public int CheckingTime
        {
            get { return checkingTime; }
            set
            {
                checkingTime = value;
                netChecker.Interval = checkingTime;
            }
        }

        /// <summary>
        /// 是否连接到了网络
        /// </summary>
        public bool IfURConnected
        {
            get { return ifURConnected; }
        }

        /// <summary>
        /// 是否临近奇异点
        /// </summary>
        public bool IfNearSingularPoint
        {
            get { return ifNearSingularPoint; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="RobotEdition">机械臂类型</param>
        /// <param name="ProgramType">机器人程序类型</param>
        /// <param name="SensorType">传感器型号</param>
        /// <param name="URIP">UR的IP地址</param>
        /// <param name="OPTOIP">OPTO的IP地址</param>
        /// <param name="LocalIPForOPTO">连接OPTO的本地IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="IfLoose">是否放松起始超时时间</param>
        /// <param name="NetCheckingTime">检查网络间隔时间</param>
        /// <param name="ForceAgentEnable">是否使用力信号代理</param>
        /// <param name="CurrentProtect">是否启动电流保护</param>
        /// <param name="ForceProtect">是否启动力超限保护</param>
        /// <param name="ToolIOAllow">是否使能工具IO输入</param>
        /// <param name="CurrentProtectLimits">电流保护极限</param>
        /// <param name="ForceProtectLimits">力保护极限</param>
        /// <param name="HangedOrNot">安装方式，是否倒装</param>
        /// <param name="DigitalVoltage">工具IO输出电压</param>
        /// <param name="GravityFactors">重力修正因子</param>
        /// <param name="GravityMethod">重力修正方式</param>
        /// <param name="MaxHalfRange">精标定探头最大半角度范围</param>
        /// <param name="RollPunctureFlag">穿刺姿态标号</param>
        /// <param name="TcpCartesianPoint">TCP安装坐标</param>
        /// <param name="GravityOfTool">工具重力</param>
        /// <param name="XCordinateForToolMass">工具重心X坐标</param>
        /// <param name="YCordinateForToolMass">工具重心Y坐标</param>
        /// <param name="ZCordinateForToolMass">工具重心Z坐标</param>
        /// <param name="FrequencySampling">采用频率，默认250Hz</param>
        /// <param name="FilterSampling">截断频率，默认4，即15Hz</param>
        /// <param name="AmountSampling">要采集的数据个数，默认0，即无穷个</param>
        public URDataProcessor(RobotType RobotEdition,
                                              RobotProgramType ProgramType,
                                              OPTODataProcessor.SensorType SensorType,
                                              string URIP,
                                              string OPTOIP,
                                              string LocalIPForOPTO,
                                              int TimeOut,
                                              bool IfLoose,
                                              int NetCheckingTime,
                                              bool ForceAgentEnable,
                                              bool CurrentProtect,
                                              bool ForceProtect,
                                              bool ToolIOAllow,
                                              double[] CurrentProtectLimits,
                                              double[] ForceProtectLimits,
                                              bool HangedOrNot,
                                              int DigitalVoltage,
                                              double[,] GravityFactors,
                                              ForceModifiedMode GravityMethod,
                                              double MaxHalfRange,
                                              byte RollPunctureFlag,
                                              double[] TcpCartesianPoint,
                                              double GravityOfTool,
                                              double XCordinateForToolMass = 0,
                                              double YCordinateForToolMass = 0,
                                              double ZCordinateForToolMass = 0,
                                              int FrequencySampling = 250,
                                              int FilterSampling = 4,
                                              int AmountSampling = 0)
            : base(TcpCartesianPoint,
                      CurrentProtect,
                      ForceProtect,
                      ToolIOAllow,
                      CurrentProtectLimits,
                      ForceProtectLimits,
                      ProgramType)
        {
            robotVersion = RobotEdition;
            switch (robotVersion)
            {
                case RobotType.CBUR3:
                case RobotType.CBUR5:
                    shoulderSingularDistance = 0.16;
                    break;

                default:
                    break;
            }
            baseController = new UR29999Connector();
            commandSender = new UR30002Connector(HangedOrNot, DigitalVoltage, TcpCartesianPoint, GravityOfTool, XCordinateForToolMass, YCordinateForToolMass, ZCordinateForToolMass);
            servoSender = new UR30004Connector();
            modbusSender = new UR502Connector();
            forceAgent = new OPTODataProcessor(SensorType, FrequencySampling, FilterSampling, AmountSampling);

            modifiedParameter = (double[,])GravityFactors.Clone();
            modifiedMethod = GravityMethod;
            maxAngleRange = MaxHalfRange;
            rollFlag = RollPunctureFlag;

            ifUseForceAgent = ForceAgentEnable;
            ipOfUR = URIP;
            ipOfOPTO = OPTOIP;
            ipOfLocalForOPTO = LocalIPForOPTO;
            timeOut = TimeOut;
            ifLoose = IfLoose;
            checkingTime = NetCheckingTime;

            forceAgent.OnSendOPTOConnectedBroken += new OPTODataProcessor.SendVoid(StopListenFromURThread);

            NonServoMotionTotalWorkInitialization();
            ServoMotionTotalWorkInitialization();

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "UR communication interface is opened.");
        }

        /// <summary>
        /// 打开网络侦测器
        /// </summary>
        public void StartNetChecking()
        {
            CheckTimerSetting(); // 设置并打开网络侦测
            netChecker.Start();
        }

        /// <summary>
        /// 设置网络侦测定时器
        /// </summary>
        protected void CheckTimerSetting()
        {
            netChecker.Interval = checkingTime;
            netChecker.AutoReset = true;
            netChecker.Elapsed += new System.Timers.ElapsedEventHandler(NetCheckerElapsed);
        }

        /// <summary>
        /// 网络侦测定时函数，定时检测网络是否通畅
        /// </summary>
        protected virtual void NetCheckerElapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            bool lockTaken = false;
            try
            {
                lockTaken = Monitor.TryEnter(netCheckLocker);
                if (lockTaken)
                {
                    Ping pingUR = new Ping();
                    int pingTime = (int)(((double)checkingTime * 0.8) / 2.0);
                    PingReply pingURReply = pingUR.Send(ipOfUR, pingTime);
                    if (pingURReply.Status == IPStatus.Success)
                    {
                        if (ifUseForceAgent)
                        {
                            Ping pingOPTO = new Ping();
                            PingReply pingOPTOReply = pingOPTO.Send(ipOfOPTO, pingTime);

                            bool ifConnectionEstablishSuccess = false;
                            if (pingOPTOReply.Status == IPStatus.Success)
                            {
                                ifConnectionEstablishSuccess = CreatCommunicationConnect(ifLoose, true);
                            }
                            else
                            {
                                ifConnectionEstablishSuccess = CreatCommunicationConnect(ifLoose, false);
                            }

                            if (ifConnectionEstablishSuccess)
                            {
                                netChecker.Stop();
                            }
                        }
                        else
                        {
                            if (CreatCommunicationConnect(ifLoose, false))
                            {
                                netChecker.Stop();
                            }
                        }
                    }
                }
                else return;
            }
            finally
            {
                if (lockTaken)
                {
                    Monitor.Exit(netCheckLocker);
                }
            }
        }

        /// <summary>
        /// 停止网络侦测定时器
        /// </summary>
        public void CheckTimerStop()
        {
            netChecker.Stop();
        }

        /// <summary>
        /// 创建通讯连接并开始监听相关端口
        /// </summary>
        /// <param name="IfLoose">是否放松起始超时时间</param>
        /// <param name="IfOPTOAttachable">OPTO是否侦测得到</param>
        /// <returns>通讯连接是否成功</returns>
        public bool CreatCommunicationConnect(bool IfLoose, bool IfOPTOAttachable = true)
        {
            if (ifUseForceAgent && !IfOPTOAttachable)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "49152 port can not be attached.");
                return false;
            }

            if (ifUseForceAgent)
            {
                try
                {
                    forceAgent.Creat49152Client(ipOfLocalForOPTO, ipOfOPTO, timeOut);
                }
                catch (Exception ex)
                {
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "49152 port connection establishing failed.", ex);
                    ifURConnected = false;
                    return false;
                }
            }

            try
            {
                baseController.Creat29999Client(ipOfUR, timeOut);
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "29999 port connection establishing failed.", ex);
                if (ifUseForceAgent)
                {
                    try
                    {
                        forceAgent.Close49152Client();
                    }
                    catch { }
                }
                ifURConnected = false;
                return false;
            }

            try
            {
                commandSender.Creat30002Client(ipOfUR, timeOut);
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "30002 port connection establishing failed.", ex);
                try
                {
                    if (ifUseForceAgent)
                    {
                        forceAgent.Close49152Client();
                    }
                    baseController.Close29999Client();
                }
                catch { }
                ifURConnected = false;
                return false;
            }

            try
            {
                if (ifUse30004Port)
                {
                    servoSender.Creat30004Client(ipOfUR, timeOut, IfLoose);
                }
                else
                {
                    modbusSender.Creat502Client(ipOfUR, timeOut);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "30004 or 502 port connection establishing failed.", ex);
                try
                {
                    if (ifUseForceAgent)
                    {
                        forceAgent.Close49152Client();
                    }
                    baseController.Close29999Client();
                    commandSender.Close30002Client();
                }
                catch { }
                ifURConnected = false;
                return false;
            }

            try
            {
                Creat30003Client(ipOfUR, timeOut, IfLoose);
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "30004 port connection establishing failed.", ex);
                try
                {
                    if (ifUseForceAgent)
                    {
                        forceAgent.Close49152Client();
                    }
                    baseController.Close29999Client();
                    commandSender.Close30002Client();
                    if (ifUse30004Port)
                    {
                        servoSender.Close30004Client();
                    }
                    else
                    {
                        modbusSender.Close502Client();
                    }
                }
                catch { }
                ifURConnected = false;
                return false;
            }

            ifURConnected = true;
            if (ifUseForceAgent) forceAgent.CreatListenFromOPTOTask();
            CreatListenFromURTask();

            OnSendURBrokenOrConnected((short)NetConnection.Connected);
            return true;
        }

        /// <summary>
        /// 主动关闭所有通讯连接
        /// </summary>
        public void ActiveBreakCommunicationConncect()
        {
            ifActiveCloseConnection = true;

            if (ifUseForceAgent)
            {
                forceAgent.StopListenFromOPTOThread();
            }
            else
            {
                StopListenFromURThread();
            }
        }

        /// <summary>
        /// 通过基本控制器发送上电指令
        /// </summary>
        public void SendURBaseControllerPowerOn()
        {
            try
            {
                if (ifURConnected)
                {
                    baseController.SendPowerOn();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending poweroff at 29999.", ex);
            }
        }

        /// <summary>
        /// 通过基本控制器发送开闸指令
        /// </summary>
        public void SendURBaseControllerBrakeRelease()
        {
            try
            {
                if (ifURConnected)
                {
                    baseController.SendBrakeRelease();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending brakerelease at 29999.", ex);
            }
        }

        /// <summary>
        /// 通过基本控制器发送断电指令
        /// </summary>
        public void SendURBaseControllerPowerOff()
        {
            try
            {
                if (ifURConnected)
                {
                    baseController.SendPowerOff();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending poweroff at 29999.", ex);
            }
        }

        /// <summary>
        /// 通过基本控制器发送机械臂和控制箱关机指令
        /// </summary>
        public void SendURBaseControllerShutDown()
        {
            try
            {
                if (ifURConnected)
                {
                    baseController.SendShutDown();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending shutdown at 29999.", ex);
            }
        }

        /// <summary>
        /// 通过基本控制器发送机械臂停止运行运动程序指令
        /// </summary>
        public void SendURBaseControllerStop()
        {
            try
            {
                if (ifURConnected)
                {
                    baseController.SendStop();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending stop at 29999.", ex);
            }
        }

        /// <summary>
        /// 设置安装方式
        /// </summary>
        /// <param name="InstallHanged">是否倒立安装</param>
        public void SetInstallation(bool InstallHanged)
        {
            commandSender.IfHanged = InstallHanged;
        }

        /// <summary>
        /// 设置工具IO电压
        /// </summary>
        /// <param name="VoltageNeed">所需的工具IO电压</param>
        public void SetVoltageLevel(int VoltageNeed)
        {
            commandSender.VoltageLevel = VoltageNeed;
        }

        /// <summary>
        /// 设置工具TCP相对末端法兰的坐标
        /// </summary>
        /// <param name="GivenTCP">所给的TCP坐标</param>
        public void SetToolTCP(double[] GivenTCP)
        {
            commandSender.TCPCordinate = GivenTCP;
            FlangeToTcp = GivenTCP;
        }

        /// <summary>
        /// 设置工具的重量，相对机械臂末端而言
        /// </summary>
        /// <param name="GivenGravity">所给的工具重量</param>
        public void SetToolGravity(double GivenGravity)
        {
            commandSender.ToolGravity = GivenGravity;
        }

        /// <summary>
        /// 设置工具重心位置，相对末端法兰而言
        /// </summary>
        /// <param name="GivenGravityPositions">所给的工具重心位置</param>
        public void SetToolGravityPositions(double[] GivenGravityPositions)
        {
            commandSender.ToolBaryCenterX = GivenGravityPositions[0];
            commandSender.ToolBaryCenterY = GivenGravityPositions[1];
            commandSender.ToolBaryCenterZ = GivenGravityPositions[2];
        }

        /// <summary>
        /// 设置工具的重力修正因子和方式
        /// </summary>
        /// <param name="GravityFactors">重力修正因子</param>
        /// <param name="GravityMethod">重力修正方式</param>
        public void SetToolGravityModify(double[,] GravityFactors, ForceModifiedMode GravityMethod)
        {
            ModifiedParameter = GravityFactors;
            ModifiedMethod = GravityMethod;
        }

        /// <summary>
        /// 通过指令发送器发送工具线性移动指令
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendURCommanderMoveL(double[] AimToolPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMoveL(AimToolPosition, a, v, t, r);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending movel at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送工具线性移动指令，输入关节角度值
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendURCommanderMoveLViaJ(double[] AimJointPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMoveLViaJ(AimJointPosition, a, v, t, r);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending movelviaj at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送关节线性移动指令
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendURCommanderMoveJ(double[] AimJointPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMoveJ(AimJointPosition, a, v, t, r);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending movej at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送关节线性移动指令，输入目标位置值
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendURCommanderMoveJViaL(double[] AimToolPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMoveJViaL(AimToolPosition, a, v, t, r);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending movejvial at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送工具圆周运动指令
        /// </summary>
        /// <param name="ViaToolPosition">途径位置数组</param>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendURCommanderMoveC(double[] ViaToolPosition, double[] AimToolPosition, double a = 1.2, double v = 0.25, double r = 0)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMoveC(ViaToolPosition, AimToolPosition, a, v, r);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending movec at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送工具定向移动指令
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组，包括方向</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        public void SendURCommanderSpeedL(double[] VelocityDirection, double a = 1.2, double t = 60)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendSpeedL(VelocityDirection, a, t);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending speedl at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送关节定向移动指令
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        public void SendURCommanderSpeedJ(double[] VelocityDirection, double a = 1.4, double t = 60)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendSpeedJ(VelocityDirection, a, t);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending speedj at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送工具线性停止指令
        /// </summary>
        /// <param name="a">制动加速度，默认1.2m/s^2</param>
        public void SendURCommanderStopL(double a = 1.2)
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendStopL(a);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending stopl at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送开始反驱示教模式
        /// </summary>
        /// <param name="DurationTime">示教模式持续时间，默认3600s</param>
        public void SendURCommanderBeginTeachMode(int DurationTime = 3600)
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendBeginTeachMode(DurationTime);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending beginteachmode at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送停止反驱示教模式
        /// </summary>
        public void SendURCommanderEndTeachMode()
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendEndTeachMode();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending endteachmode at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送暂停指令
        /// </summary>
        /// <param name="DurationTime">暂停时间，单位s</param>
        public void SendURCommanderPause(int DurationTime)
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendPause(DurationTime);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending pause at 30002.", ex);
            }
        }

        /// <summary>
        /// 写入控制程序
        /// </summary>
        /// <param name="WriteStrs">被写入控制程序的代码列表</param>
        public void WriteStringToControlCode(List<string> WriteStrs)
        {
            using (FileStream fileStream = new FileStream(commandSender.scriptCommand.ControllerCodePath, FileMode.Create))
            {
                using (StreamWriter streamWriter = new StreamWriter(fileStream))
                {
                    streamWriter.Write(string.Join("\r\n", WriteStrs.ToArray()));
                    streamWriter.Flush();
                }
            }
        }

        /// <summary>
        /// 写入控制程序，只写入伺服运动部分
        /// </summary>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void WriteStringToControlCode(double ControlPeriod = 0.008, double LookAheadTime = 0.1, double Gain = 200)
        {
            List<string> WriteStrs = new List<string>(100);
            if (ifUse30004Port)
            {
                WriteStrs.Add("global tcpPosition = p[0.0,0.0,0.0,0.0,0.0,0.0]");
                WriteStrs.Add("while (True):");
                WriteStrs.Add("  tcpPosition[0] = read_input_float_register(0)");
                WriteStrs.Add("  tcpPosition[1] = read_input_float_register(1)");
                WriteStrs.Add("  tcpPosition[2] = read_input_float_register(2)");
                WriteStrs.Add("  tcpPosition[3] = read_input_float_register(3)");
                WriteStrs.Add("  tcpPosition[4] = read_input_float_register(4)");
                WriteStrs.Add("  tcpPosition[5] = read_input_float_register(5)");
                WriteStrs.Add("  servoj(get_inverse_kin(tcpPosition), t = " + ControlPeriod.ToString("0.000") + ", lookahead_time = " + LookAheadTime.ToString("0.00") + ", gain = " + Gain.ToString("0") + ")");
                WriteStrs.Add("end");
            }
            else
            {
                WriteStrs.Add("global tcpPosition = p[0.0,0.0,0.0,0.0,0.0,0.0]");
                WriteStrs.Add("global tcpX = [0.0,0.0,0.0]");
                WriteStrs.Add("global tcpY = [0.0,0.0,0.0]");
                WriteStrs.Add("global tcpZ = [0.0,0.0,0.0]");
                WriteStrs.Add("while (True):");
                WriteStrs.Add("  tcpX[0] = read_port_register(130)");
                WriteStrs.Add("  tcpX[1] = read_port_register(131)");
                WriteStrs.Add("  tcpX[2] = read_port_register(132)");
                WriteStrs.Add("  tcpY[0] = read_port_register(133)");
                WriteStrs.Add("  tcpY[1] = read_port_register(134)");
                WriteStrs.Add("  tcpY[2] = read_port_register(135)");
                WriteStrs.Add("  tcpZ[0] = read_port_register(136)");
                WriteStrs.Add("  tcpZ[1] = read_port_register(137)");
                WriteStrs.Add("  tcpZ[2] = read_port_register(138)");
                WriteStrs.Add("  if (tcpX[2] > 20000):");
                WriteStrs.Add("    tcpPosition[0] = -(tcpX[0] + tcpX[1] + tcpX[2] - 40000.0) / 100000.0");
                WriteStrs.Add("  else:");
                WriteStrs.Add("    tcpPosition[0] = (tcpX[0] + tcpX[1] + tcpX[2]) / 100000.0");
                WriteStrs.Add("  end");
                WriteStrs.Add("  if (tcpY[2] > 20000):");
                WriteStrs.Add("    tcpPosition[1] = -(tcpY[0] + tcpY[1] + tcpY[2] - 40000.0) / 100000.0");
                WriteStrs.Add("  else:");
                WriteStrs.Add("    tcpPosition[1] = (tcpY[0] + tcpY[1] + tcpY[2]) / 100000.0");
                WriteStrs.Add("  end");
                WriteStrs.Add("  if (tcpZ[2] > 20000):");
                WriteStrs.Add("    tcpPosition[2] = -(tcpZ[0] + tcpZ[1] + tcpZ[2] - 40000.0) / 100000.0");
                WriteStrs.Add("  else:");
                WriteStrs.Add("    tcpPosition[2] = (tcpZ[0] + tcpZ[1] + tcpZ[2]) / 100000.0");
                WriteStrs.Add("  end");
                WriteStrs.Add("  tcpPosition[3]= (read_port_register(140) * 2.0 - 1.0) * read_port_register(139) / 10000.0");
                WriteStrs.Add("  tcpPosition[4]= (read_port_register(142) * 2.0 - 1.0) * read_port_register(141) / 10000.0");
                WriteStrs.Add("  tcpPosition[5]= (read_port_register(144) * 2.0 - 1.0) * read_port_register(143) / 10000.0");
                WriteStrs.Add("  servoj(get_inverse_kin(tcpPosition), t = " + ControlPeriod.ToString("0.000") + ", lookahead_time = " + LookAheadTime.ToString("0.00") + ", gain = " + Gain.ToString("0") + ")");
                WriteStrs.Add("end");
            }

            WriteStringToControlCode(WriteStrs);
        }

        /// <summary>
        /// 通过指令发送器发送下位机控制指令
        /// </summary>
        public void SendURCommanderControllerCode()
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendControllerCode();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending controlcode at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送自定义运动指令
        /// </summary>
        /// <param name="MotionCommand">自定义运动指令字符串</param>
        public void SendURCommanderMotionCommand(List<string> MotionCommand)
        {
            try
            {
                if (ifURConnected && !ifNearSingularPoint)
                {
                    commandSender.SendMotionCommand(MotionCommand);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending custom motioncommand at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送自定义非运动指令
        /// </summary>
        /// <param name="NonMotionCommand">自定义非运动指令字符串</param>
        public void SendURCommanderNonMotionCommand(List<string> NonMotionCommand)
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendNonMotionCommand(NonMotionCommand);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending custom nonmotioncommand at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过指令发送器发送基础设置
        /// </summary>
        public void SendURCommanderBaseSetting()
        {
            try
            {
                if (ifURConnected)
                {
                    commandSender.SendBaseSetting();
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending basesetting at 30002.", ex);
            }
        }

        /// <summary>
        /// 通过伺服发送器设置寄存器用以存放发送的工具坐标
        /// </summary>
        /// <param name="inputSetupStr">需要使用的寄存器</param>
        public void SendURServorInputSetup(string inputSetupStr = "input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5")
        {
            try
            {
                if (ifURConnected)
                {
                    servoSender.ToolPositionInputSetup(inputSetupStr);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending inputsetup at 30004.", ex);
            }
        }

        /// <summary>
        /// 通过伺服发送器发送工具坐标到设置好的寄存器
        /// </summary>
        /// <param name="InputToolPosition">被发送的工具坐标</param>
        public void SendURServorInputDatas(double[] InputToolPosition)
        {
            try
            {
                if (ifURConnected)
                {
                    servoSender.ToolPositionInputDatas(InputToolPosition);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending inputdata at 30004.", ex);
            }
        }

        /// <summary>
        /// 通过Modbus发送器发送工具坐标到指定的寄存器
        /// </summary>
        /// <param name="InputToolPosition">被发送的工具坐标</param>
        public void SendURModbusInputDatas(double[] InputToolPosition)
        {
            try
            {
                if (ifURConnected)
                {
                    modbusSender.WriteRegister(InputToolPosition);
                }
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Connection exception when sending inputdata at 502.", ex);
            }
        }

        /// <summary>
        /// 通过力信号代理设置力信号采样频率
        /// </summary>
        /// <param name="FrequencyOfSampling">采样频率</param>
        public void SetOPTOSampleFrequency(int FrequencyOfSampling)
        {
            if (ifUseForceAgent) forceAgent.SampleFrequency = FrequencyOfSampling;
        }

        /// <summary>
        /// 通过力信号代理设置力信号截断频率
        /// </summary>
        /// <param name="FilterOfFrequency">截断频率</param>
        public void SetOPTOFrequencyFilter(int FilterOfFrequency)
        {
            if (ifUseForceAgent) forceAgent.FrequencyFilter = FilterOfFrequency;
        }

        /// <summary>
        /// 通过力信号代理设置力信号采集个数
        /// </summary>
        /// <param name="CountOfSampling">采集个数，0代表无穷个</param>
        public void SetOPTOSampleCount(int CountOfSampling)
        {
            if (ifUseForceAgent) forceAgent.SampleCount = CountOfSampling;
        }

        /// <summary>
        /// 通过力信号代理设置力信号零点开关，阻塞式
        /// </summary>
        /// <param name="SwitchState">是否置零数据</param>
        /// <param name="IntervalTime">开关间隔时间，默认12ms</param>
        public void SetOPTOBias(bool SwitchState = false, int IntervalTime = 12)
        {
            if (ifUseForceAgent && forceAgent.IfOPTOConnected)
            {
                forceAgent.SwitchBiasOpenOrClose(SwitchState, IntervalTime);
                Task tempTask = Task.Run(new Action(() =>
                {
                    Thread.Sleep(500);
                    double[][] getForces = ContinuousOriginalFlangeForces;
                    int numForces = getForces.Count();
                    double[] fx = new double[numForces];
                    double[] fy = new double[numForces];
                    double[] fz = new double[numForces];
                    double[] tx = new double[numForces];
                    double[] ty = new double[numForces];
                    double[] tz = new double[numForces];
                    for (int i = 0; i < numForces; i++)
                    {
                        fx[i] = getForces[i][0];
                        fy[i] = getForces[i][1];
                        fz[i] = getForces[i][2];
                        tx[i] = getForces[i][3];
                        ty[i] = getForces[i][4];
                        tz[i] = getForces[i][5];
                    }
                    lock (zeroForceLocker)
                    {
                        zeroForceBias[0] = -URMath.GaussAverage(fx, 4);
                        zeroForceBias[1] = -URMath.GaussAverage(fy, 4);
                        zeroForceBias[2] = -URMath.GaussAverage(fz, 4);
                        zeroForceBias[3] = -URMath.GaussAverage(tx, 5);
                        zeroForceBias[4] = -URMath.GaussAverage(ty, 5);
                        zeroForceBias[5] = -URMath.GaussAverage(tz, 5);
                    }
                    OnSendZeroedForceCompeleted();
                }));
            }
        }

        /// <summary>
        /// 接收实时控制状态并解析，已重载
        /// </summary>
        /// <returns>本次接收是否成功</returns>
        protected override bool UnpackRecievedDatas()
        {
            try
            {
                return base.UnpackRecievedDatas();
            }
            catch (Exception ex)
            {
                listenCancelSource.Cancel();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "UR listener canceled.", ex);
                return false;
            }
        }

        /// <summary>
        /// 收到准确的UR实时数据后需要做的工作，已重载
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
        /// A. 状态检查，奇异点临近检查
        /// </summary>
        protected override void BaseWorkAfterRecievedCorrectDatas()
        {
            base.BaseWorkAfterRecievedCorrectDatas();

            try
            {
                if (!ifUse30004Port && ifKeepSendModbusPackage)
                {
                    if (modbusCount < modbusMaxCount)
                    {
                        modbusCount++;
                    }
                    else
                    {
                        modbusSender.WriteRegister((double[])positionsTcpActual.Clone());
                        modbusCount = 0;
                    }
                }
            }
            catch (Exception ex)
            {
                listenCancelSource.Cancel();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "UR listener canceled.", ex);
            }

            // A. 检查是否临近奇异点
            if (ifOpenSingularCheck)
            {
                DealWithNearSingularPoint(MethodBase.GetCurrentMethod().DeclaringType.FullName, true);
            }
        }

        /// <summary>
        /// 判断是否临近奇异点
        /// </summary>
        /// <returns>返回判断结果</returns>
        protected short JudgeIfNearSingularPoint()
        {
            // 肩部奇异点终止
            if (Math.Sqrt(Math.Pow(positionsFlangeActual[0], 2) + Math.Pow(positionsFlangeActual[1], 2)) < shoulderSingularDistance)
            {
                return 1;
            }

            // 肘部奇异点终止
            if (Math.Abs(positionsJointActual[2]) < 0.262)
            {
                return 2;
            }

            // 腕部奇异点终止
            if (Math.Abs(positionsJointActual[4]) < 0.087 || Math.Abs(positionsJointActual[4]) > 3.055)
            {
                return 3;
            }

            return 0;
        }

        /// <summary>
        /// 处理临近奇异点工作
        /// </summary>
        /// <param name="ClassName">处理工作所在的类名</param>
        /// <param name="IfSelfJudge">是否自动判断和处理临近奇异点工作</param>
        /// <returns>返回判断结果</returns>
        protected virtual short DealWithNearSingularPoint(string ClassName = "", bool IfSelfJudge = false)
        {
            short nearResult = JudgeIfNearSingularPoint();

            if (IfSelfJudge)
            {
                if (modeRobotRunning && nearResult != 0)
                {
                    if (!ifNearSingularPoint)
                    {
                        ifNearSingularPoint = true;

                        SendURCommanderStopL();
                        NonServoMotionTotalAbort();

                        string content = "";
                        switch (nearResult)
                        {
                            case 1:
                                content = "Close to shoulder singular point.";
                                break;
                            case 2:
                                content = "Close to elbow singular point.";
                                break;
                            case 3:
                                content = "Close to wrist singular point.";
                                break;
                            default:
                                break;
                        }
                        Logger.HistoryPrinting(Logger.Level.WARN, ClassName, content);

                        OnSendNearSingularPoint(nearResult);
                    }
                }
                else
                {
                    ifNearSingularPoint = false;
                }
            }

            return nearResult;
        }

        /// <summary>
        /// 打开伪奇异点状态模式，假设不在奇异点附近并关闭奇异点检查
        /// </summary>
        public virtual void OpenFakeSingularPointStatus()
        {
            ifOpenSingularCheck = false; // 暂时关闭奇异点检查
            Task.Run(new Action(() =>
            {
                Thread.Sleep(64);
                ifNearSingularPoint = false; // 暂时假设不在奇异点附近，使得运动指令有效
            }));
        }

        /// <summary>
        /// 关闭伪奇异点状态模式，假设处于奇异点附近并打开奇异点检查
        /// </summary>
        public virtual void CloseFakeSingularPointStatus()
        {
            ifNearSingularPoint = true; // 假设在奇异点附近，使得运动指令无效
            Task.Run(new Action(() =>
            {
                Thread.Sleep(64);
                ifOpenSingularCheck = true; // 打开奇异点检查
            }));
        }

        /// <summary>
        /// 拷贝力信号数据，已重载
        /// </summary>
        protected override void CopyForceData()
        {
            if (ifUseForceAgent)
            {
                originalFlangeForces = forceAgent.GetOrignalForceInformation();
                lock (zeroForceLocker)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        zeroedOriginalFlangeForces[i] = originalFlangeForces[i] + zeroForceBias[i];
                    }
                }
            }
        }

        /// <summary>
        /// 紧急事件处理，已重载
        /// </summary>
        /// <param name="EmergencyState">紧急状态</param>
        protected override void DealWithEmergency(RobotEmergency EmergencyState)
        {
            if (EmergencyState == RobotEmergency.CurrentOverflow || EmergencyState == RobotEmergency.ForceOverflow)
            {
                SendURBaseControllerStop();
                Task.Run(new Action(() =>
                {
                    Thread.Sleep(250);
                    SendURBaseControllerPowerOff();
                }));
            }
        }

        /// <summary>
        /// 处理工具IO输入响应事件，已重载
        /// </summary>
        /// <param name="IONumber">工具IO号</param>
        protected override void DealWithIOInput(int IONumber)
        {
            if (IONumber == 0)
            {
                // 0号工具IO执行程序
            }
            else if (IONumber == 1)
            {
                // 1号工具IO执行程序
            }
        }

        /// <summary>
        /// 探头去重力，已重载
        /// </summary>
        /// <param name="ToolPosition">工具位姿</param>
        /// <param name="FlangePosition">法兰位姿</param>
        /// <param name="OriginalForce">原始力信息</param>
        /// <param name="ModifiedParameter">修正因子</param>
        /// <param name="ModifiedMethod">修正方式</param>
        /// <returns>去重力后的力信息</returns>
        protected override double[] CalculateForceWithoutGravity(double[] ToolPosition, double[] FlangePosition, double[] OriginalForce, double[,] ModifiedParameter, ForceModifiedMode ModifiedMethod)
        {
            switch (ModifiedMethod)
            {
                case ForceModifiedMode.NoModified:
                    {
                        return (double[])OriginalForce.Clone();
                    }
                case ForceModifiedMode.ProbePrecise:
                    {
                        double[] toolYAxis = YDirectionOfTcpAtBaseReference(ToolPosition);
                        double angleWithXAxis = (Math.Abs(toolYAxis[2]) > 1.0) ? Math.Asin((double)Math.Sign(toolYAxis[2])) : Math.Asin(toolYAxis[2]);
                        if (!commandSender.IfHanged) angleWithXAxis = -angleWithXAxis;
                        
                        int halfNum = (ModifiedParameter.GetLength(1) - 1) / 2;
                        double step = maxAngleRange / (double)halfNum;

                        int lowBound = 0;
                        int upBound = 1;
                        double proportion = 0.0;
                        if (angleWithXAxis > 0)
                        {
                            lowBound = (int)Math.Floor(angleWithXAxis / step);
                            if (lowBound > halfNum - 1)
                            {
                                lowBound = halfNum - 1;
                            }
                            proportion = (angleWithXAxis - lowBound * step) / step;

                            if (lowBound > 0)
                            {
                                lowBound += halfNum;
                                upBound = lowBound + 1;
                            }
                            else
                            {
                                upBound += halfNum;
                            }
                        }
                        else if (angleWithXAxis < 0)
                        {
                            lowBound = (int)Math.Floor(-angleWithXAxis / step);
                            if (lowBound > halfNum - 1)
                            {
                                lowBound = halfNum - 1;
                            }
                            proportion = (-angleWithXAxis - lowBound * step) / step;

                            upBound = lowBound + 1;
                        }
                        else
                        {
                            return new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                        }

                        double[] noGravityForce = new double[6];
                        for (int i = 0; i < 6; i++)
                        {
                            noGravityForce[i] = OriginalForce[i] - (ModifiedParameter[i, lowBound] * (1.0 - proportion) + ModifiedParameter[i, upBound] * proportion);
                        }

                        return (double[])noGravityForce.Clone();
                    }
                case ForceModifiedMode.PuncturePrecise:
                    {
                        // 判断姿态
                        double[] toolZAxis = ZDirectionOfTcpAtBaseReference(ToolPosition);
                        double angleToZAxis = (Math.Abs(toolZAxis[2]) > 1.0) ? Math.Acos((double)Math.Sign(toolZAxis[2])) : Math.Acos(toolZAxis[2]);
                        double angleRotateToZAxis = angleToZAxis - (Math.PI / 2.0 + minPitchAngle);
                        int totalNum = (ModifiedParameter.GetLength(1) - 1) / 4 - 1;
                        double step = (maxPitchAngle - minPitchAngle) / (double)totalNum;
                        double num = angleRotateToZAxis / step;

                        int lowBound = 1;
                        int upBound = 2;
                        double proportion = 0.0;
                        if (num < 0.0)
                        {
                            lowBound = 1;
                            upBound = lowBound + 1;
                            proportion = angleRotateToZAxis / step;
                        }
                        else if (num > totalNum)
                        {
                            lowBound = totalNum;
                            upBound = lowBound + 1;
                            proportion = (angleRotateToZAxis - (lowBound - 1) * step) / step;
                        }
                        else
                        {
                            lowBound = (int)Math.Ceiling(num);
                            upBound = lowBound + 1;
                            proportion = (angleRotateToZAxis - (lowBound - 1) * step) / step;
                        }
                        lowBound = 4 * lowBound - 3 + rollFlag;
                        upBound = 4 * upBound - 3 + rollFlag;

                        double[] noGravityForce = new double[6];
                        for (int i = 0; i < 6; i++)
                        {
                            noGravityForce[i] = OriginalForce[i] - (ModifiedParameter[i, lowBound] * (1.0 - proportion) + ModifiedParameter[i, upBound] * proportion);
                        }

                        return (double[])noGravityForce.Clone();
                    }
                case ForceModifiedMode.ProbeWhole:
                    {





                        double[] noGravityForce = new double[6];



                        return (double[])noGravityForce.Clone();
                    }
                default:
                    {
                        return new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                    }
            }
        }

        /// <summary>
        /// 利用UR反馈数据的频率作为定时器执行一定的功能，已重载
        /// </summary>
        protected override void WorkBasedOnListeningAsTimer()
        {
            try
            {
                NonServoMotionTotalWork();
                ServoMotionTotalWork();
            }
            catch (Exception ex)
            {
                listenCancelSource.Cancel();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "UR listener canceled.", ex);
            }
        }

        /// <summary>
        /// UR通讯连接中断，已重载
        /// </summary>
        protected override void URConnectionBroken()
        {
            bool ifRestartNetChecker = true;

            // 发起UR连接断开事件
            if (ifActiveCloseConnection)
            {
                ifRestartNetChecker = false;
                OnSendURBrokenOrConnected((short)NetConnection.ActiveBroken);
                ifActiveCloseConnection = false;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Network connection broken actively.");
            }
            else
            {
                OnSendURBrokenOrConnected((short)NetConnection.Broken);
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Network connection broken passively.");
            }

            // 断开所有UR连接
            try
            {
                baseController.Close29999Client();
            }
            catch { }

            try
            {
                commandSender.Close30002Client();
            }
            catch { }

            try
            {
                if (ifUse30004Port)
                {
                    servoSender.Close30004Client();
                }
                else
                {
                    modbusSender.Close502Client();
                }
            }
            catch { }

            try
            {
                Close30003Client();
            }
            catch { }

            ifURConnected = false;
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "UR connection crashed.");

            // 使用力信号代理并且它没关闭
            if (ifUseForceAgent && forceAgent.IfOPTOConnected)
            {
                // 开新任务执行OPTO关闭的操作，防止死锁
                Task.Run(new Action(() =>
                {
                    forceAgent.StopListenFromOPTOThread();
                }));
            }

            if (ifRestartNetChecker)
            {
                // 至少在1000ms后打开网络侦测器
                Task.Run(new Action(() =>
                {
                    Thread.Sleep(1000);
                    netChecker.Start();
                }));
            }
        }

        /// <summary>
        /// 探头精确力标定
        /// </summary>
        /// <param name="InitialPosition">进行力清零的初始位置</param>
        /// <param name="RotationAxis">绕轴旋转</param>
        /// <param name="HalfTestNum">半扇记录点数</param>
        /// <param name="HalfRange">半扇角度范围</param>
        /// <param name="IsZeroed">是否已经力清零</param>
        public virtual void PreciseForceCalibrationProbe(double[] InitialPosition, char RotationAxis, int HalfTestNum, double HalfRange, bool IsZeroed = true)
        {
            maxAngleRange = HalfRange;

            Task.Run(new Action(() =>
            {
                double[,] forceRecord = new double[6, HalfTestNum * 2 + 1];

                if (!IsZeroed)
                {
                    // 先移动到初始位置
                    SendURCommanderMoveJ(InitialPosition, 0.5, 0.4);
                    Thread.Sleep(800);
                    while (ProgramState == 2.0)
                    {
                        Thread.Sleep(200);
                    }

                    // 等待机械臂稳定
                    Thread.Sleep(1000);

                    // 重置力传感器
                    SetOPTOBias(true);
                    Thread.Sleep(800);
                }

                // 记录当前的机械臂TCP坐标
                double[] zeroedPosition = PositionsTcpActual;
                for (int i = 0; i < 6; i++) // 初始位置置零
                {
                    forceRecord[i, 0] = 0.0;
                }
                OnSendPreciseCalibrationProcess((short)1);

                double Step = HalfRange / (double)HalfTestNum;
                for (int k = 0; k < HalfTestNum; k++)
                {
                    // 进行负值移动
                    switch (RotationAxis)
                    {
                        case 'x':
                        case 'X':
                            SendURCommanderMoveL(RotateByTcpXAxis(-(k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        case 'y':
                        case 'Y':
                            SendURCommanderMoveL(RotateByTcpYAxis(-(k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        case 'z':
                        case 'Z':
                            SendURCommanderMoveL(RotateByTcpZAxis(-(k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        default:
                            SendURCommanderMoveL(zeroedPosition, 0.1, 0.03);
                            break;
                    }

                    Thread.Sleep(800);
                    while (ProgramState == 2.0)
                    {
                        Thread.Sleep(200);
                    }

                    // 等待机械臂稳定
                    Thread.Sleep(1000);

                    // 测试力信号
                    Thread.Sleep(500);
                    double[][] getForces = ContinuousOriginalFlangeForces;
                    int numForces = getForces.Count();
                    double[] fx = new double[numForces];
                    double[] fy = new double[numForces];
                    double[] fz = new double[numForces];
                    double[] tx = new double[numForces];
                    double[] ty = new double[numForces];
                    double[] tz = new double[numForces];
                    for (int i = 0; i < numForces; i++)
                    {
                        fx[i] = getForces[i][0];
                        fy[i] = getForces[i][1];
                        fz[i] = getForces[i][2];
                        tx[i] = getForces[i][3];
                        ty[i] = getForces[i][4];
                        tz[i] = getForces[i][5];
                    }
                    forceRecord[0, k + 1] = URMath.GaussAverage(fx, 4);
                    forceRecord[1, k + 1] = URMath.GaussAverage(fy, 4);
                    forceRecord[2, k + 1] = URMath.GaussAverage(fz, 4);
                    forceRecord[3, k + 1] = URMath.GaussAverage(tx, 5);
                    forceRecord[4, k + 1] = URMath.GaussAverage(ty, 5);
                    forceRecord[5, k + 1] = URMath.GaussAverage(tz, 5);

                    int process = 1 + (int)((double)(k + 1) / (double)HalfTestNum * 48.0);
                    OnSendPreciseCalibrationProcess((short)process);
                }

                // 回到初始位置
                SendURCommanderMoveL(zeroedPosition, 0.1, 0.03);
                Thread.Sleep(800);
                while (ProgramState == 2.0)
                {
                    Thread.Sleep(200);
                }
                OnSendPreciseCalibrationProcess((short)51);

                for (int k = 0; k < HalfTestNum; k++)
                {
                    // 进行正值移动
                    switch (RotationAxis)
                    {
                        case 'x':
                        case 'X':
                            SendURCommanderMoveL(RotateByTcpXAxis((k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        case 'y':
                        case 'Y':
                            SendURCommanderMoveL(RotateByTcpYAxis((k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        case 'z':
                        case 'Z':
                            SendURCommanderMoveL(RotateByTcpZAxis((k + 1) * Step, zeroedPosition), 0.1, 0.03);
                            break;
                        default:
                            SendURCommanderMoveL(zeroedPosition, 0.1, 0.03);
                            break;
                    }

                    Thread.Sleep(800);
                    while (ProgramState == 2.0)
                    {
                        Thread.Sleep(200);
                    }

                    // 等待机械臂稳定
                    Thread.Sleep(1000);

                    // 测试力信号
                    Thread.Sleep(500);
                    double[][] getForces = ContinuousOriginalFlangeForces;
                    int numForces = getForces.Count();
                    double[] fx = new double[numForces];
                    double[] fy = new double[numForces];
                    double[] fz = new double[numForces];
                    double[] tx = new double[numForces];
                    double[] ty = new double[numForces];
                    double[] tz = new double[numForces];
                    for (int i = 0; i < numForces; i++)
                    {
                        fx[i] = getForces[i][0];
                        fy[i] = getForces[i][1];
                        fz[i] = getForces[i][2];
                        tx[i] = getForces[i][3];
                        ty[i] = getForces[i][4];
                        tz[i] = getForces[i][5];
                    }
                    forceRecord[0, k + HalfTestNum + 1] = URMath.GaussAverage(fx, 4);
                    forceRecord[1, k + HalfTestNum + 1] = URMath.GaussAverage(fy, 4);
                    forceRecord[2, k + HalfTestNum + 1] = URMath.GaussAverage(fz, 4);
                    forceRecord[3, k + HalfTestNum + 1] = URMath.GaussAverage(tx, 5);
                    forceRecord[4, k + HalfTestNum + 1] = URMath.GaussAverage(ty, 5);
                    forceRecord[5, k + HalfTestNum + 1] = URMath.GaussAverage(tz, 5);

                    int process = 51 + (int)((double)(k + 1) / (double)HalfTestNum * 48.0);
                    OnSendPreciseCalibrationProcess((short)process);
                }

                // 回到初始位置
                SendURCommanderMoveL(zeroedPosition, 0.1, 0.03);
                OnSendPreciseCalibrationDatas((double[,])forceRecord.Clone());

                Thread.Sleep(800);
                while (ProgramState == 2.0)
                {
                    Thread.Sleep(200);
                }
                OnSendPreciseCalibrationProcess((short)100);
            }));
        }

        /// <summary>
        /// 穿刺精确力标定
        /// </summary>
        /// <param name="InitialPosition">进行力清零的初始位置</param>
        /// <param name="TotalTestNum">总记录点数</param>
        /// <param name="IsZeroed">是否已经力清零</param>
        public virtual void PreciseForceCalibrationPuncture(double[] InitialPosition, int TotalTestNum, bool IsZeroed = true)
        {
            Task.Run(new Action(() =>
            {
                double[,] forceRecord = new double[6, TotalTestNum + 1];

                if (!IsZeroed)
                {
                    // 先移动到初始位置
                    SendURCommanderMoveJViaL(InitialPosition, 0.5, 0.4);
                    Thread.Sleep(800);
                    while (ProgramState == 2.0)
                    {
                        Thread.Sleep(200);
                    }

                    // 等待机械臂稳定
                    Thread.Sleep(1000);

                    // 重置力传感器
                    SetOPTOBias(true);
                    Thread.Sleep(800);
                }

                // 回到测量原位
                double[] zAxisOfTcp = ZDirectionOfTcpAtBaseReference();
                double angleToZAxis = (Math.Abs(zAxisOfTcp[2]) > 1.0) ? Math.Acos((double)Math.Sign(zAxisOfTcp[2])) : Math.Acos(zAxisOfTcp[2]);
                double angleRotateWithYAxis = angleToZAxis - (Math.PI / 2.0 + minPitchAngle);
                double[] zeroedPosition = RotateByTcpYAxis(angleRotateWithYAxis);
                SendURCommanderMoveJViaL(zeroedPosition, 0.5, 0.4);
                Thread.Sleep(800);
                while (ProgramState == 2.0)
                {
                    Thread.Sleep(200);
                }
                OnSendPreciseCalibrationProcess((short)1);

                // 开始测量
                int eachModeNum = TotalTestNum / 4 - 1;
                double Step = (maxPitchAngle - minPitchAngle) / (double)eachModeNum;
                double accumStep = 0;
                int k = 1;

                while (accumStep < maxPitchAngle - minPitchAngle + Step / 2.0)
                {
                    for (int h = 0; h < 4; h++)
                    {
                        double[] aimPosition = RotateByTcpZAxis(Math.PI / 2.0 * h,
                                                             RotateByTcpYAxis(-accumStep, zeroedPosition));
                        SendURCommanderMoveJViaL(aimPosition, 0.5, 0.4);
                        Thread.Sleep(800);
                        while (ProgramState == 2.0)
                        {
                            Thread.Sleep(200);
                        }

                        // 等待机械臂稳定
                        Thread.Sleep(2000);

                        // 测试力信号
                        Thread.Sleep(500);
                        double[][] getForces = ContinuousOriginalFlangeForces;
                        int numForces = getForces.Count();
                        double[] fx = new double[numForces];
                        double[] fy = new double[numForces];
                        double[] fz = new double[numForces];
                        double[] tx = new double[numForces];
                        double[] ty = new double[numForces];
                        double[] tz = new double[numForces];
                        for (int i = 0; i < numForces; i++)
                        {
                            fx[i] = getForces[i][0];
                            fy[i] = getForces[i][1];
                            fz[i] = getForces[i][2];
                            tx[i] = getForces[i][3];
                            ty[i] = getForces[i][4];
                            tz[i] = getForces[i][5];
                        }
                        forceRecord[0, k] = URMath.GaussAverage(fx, 4);
                        forceRecord[1, k] = URMath.GaussAverage(fy, 4);
                        forceRecord[2, k] = URMath.GaussAverage(fz, 4);
                        forceRecord[3, k] = URMath.GaussAverage(tx, 5);
                        forceRecord[4, k] = URMath.GaussAverage(ty, 5);
                        forceRecord[5, k] = URMath.GaussAverage(tz, 5);

                        int process = 1 + (int)(accumStep / (maxPitchAngle - minPitchAngle) * 98.0);
                        OnSendPreciseCalibrationProcess((short)process);

                        k++;
                    }

                    // 原路返回，防止绕线
                    for (int h = 2; h > 0; h--)
                    {
                        double[] aimPosition = RotateByTcpZAxis(Math.PI / 2.0 * h,
                                                             RotateByTcpYAxis(-accumStep, zeroedPosition));
                        SendURCommanderMoveJViaL(aimPosition, 0.8, 0.6);
                        Thread.Sleep(800);
                        while (ProgramState == 2.0)
                        {
                            Thread.Sleep(200);
                        }
                    }

                    accumStep += Step;
                }

                // 回到初始位置
                SendURCommanderMoveJViaL(InitialPosition, 0.5, 0.4);
                OnSendPreciseCalibrationDatas((double[,])forceRecord.Clone());

                Thread.Sleep(800);
                while (ProgramState == 2.0)
                {
                    Thread.Sleep(200);
                }
                OnSendPreciseCalibrationProcess((short)100);
            }));
        }

        /// <summary>
        /// 探头完整力标定
        /// </summary>
        /// <param name="InitialPosition">进行力清零的初始位置</param>
        /// <param name="IsZeroed">是否已经力清零</param>
        public virtual void EntireForceCalibrationProbe(double[] InitialPosition, bool IsZeroed = true)
        {
            Task.Run(new Action(() =>
            {
                if (!IsZeroed)
                {
                    // 先移动到初始位置
                    SendURCommanderMoveJ(InitialPosition, 0.5, 0.4);
                    Thread.Sleep(800);
                    while (ProgramState == 2.0)
                    {
                        Thread.Sleep(200);
                    }

                    // 等待机械臂稳定
                    Thread.Sleep(1000);

                    // 重置力传感器
                    SetOPTOBias(true);
                    Thread.Sleep(800);
                }

                Thread.Sleep(1000);
                // 记录当前的机械臂TCP坐标
                double[] zeroedPosition = PositionsTcpActual;

                // 开始循环采集
                for (int loop = 1; loop < 16; loop++)
                {
                    // 初始化数据容器
                    double[,] forceRecord = new double[6, 360 / 2 * 3];

                    // 起始计量位置
                    double[] startPos = (double[])zeroedPosition.Clone();

                    for (int subLoop = 1; subLoop < 4; subLoop++)
                    {
                        double yAngle = (3 * (loop - 1) + subLoop) * URMath.Deg2Rad(2);
                        double[] thisLoopstartPos = RotateByTcpYAxis(yAngle, zeroedPosition);

                        for (int num = 0; num < 180; num++)
                        {
                            double zAngle = -URMath.Deg2Rad(180) + URMath.Deg2Rad(2) * num;
                            double[] thisLoopPos = RotateByTcpZAxis(zAngle, thisLoopstartPos);

                            // 移动到测量位置
                            SendURCommanderMoveJViaL(thisLoopPos, 0.2, 0.1);

                            #region 测量力信号
                            Thread.Sleep(800);
                            while (ProgramState == 2.0)
                            {
                                Thread.Sleep(200);
                            }

                            // 等待机械臂稳定
                            Thread.Sleep(1000);

                            // 测试力信号
                            Thread.Sleep(500+250);
                            double[][] getForces = ContinuousOriginalFlangeForces;
                            int numForces = getForces.Count();
                            double[] fx = new double[numForces];
                            double[] fy = new double[numForces];
                            double[] fz = new double[numForces];
                            double[] tx = new double[numForces];
                            double[] ty = new double[numForces];
                            double[] tz = new double[numForces];
                            for (int i = 0; i < numForces; i++)
                            {
                                fx[i] = getForces[i][0];
                                fy[i] = getForces[i][1];
                                fz[i] = getForces[i][2];
                                tx[i] = getForces[i][3];
                                ty[i] = getForces[i][4];
                                tz[i] = getForces[i][5];
                            }
                            forceRecord[0, (subLoop - 1) * 180 + num] = URMath.GaussAverage(fx, 4, true);
                            forceRecord[1, (subLoop - 1) * 180 + num] = URMath.GaussAverage(fy, 4, true);
                            forceRecord[2, (subLoop - 1) * 180 + num] = URMath.GaussAverage(fz, 4, true);
                            forceRecord[3, (subLoop - 1) * 180 + num] = URMath.GaussAverage(tx, 5, true);
                            forceRecord[4, (subLoop - 1) * 180 + num] = URMath.GaussAverage(ty, 5, true);
                            forceRecord[5, (subLoop - 1) * 180 + num] = URMath.GaussAverage(tz, 5, true);
                            #endregion

                            OnSendEntireCalibrationProcess(Convert.ToInt16(Math.Round((3 * (loop - 1) + subLoop - 1) * 2.0 +num * 2.0 / 180)));
                        }

                        #region 原路返回，防止绕线
                        double[] tempPos = (double[])thisLoopstartPos.Clone();
                        for (int round = 1; round < 3; round++)
                        {
                            double zAngle = URMath.Deg2Rad(180) - URMath.Deg2Rad(120) * round;
                            double[] thisLoopPos = RotateByTcpZAxis(zAngle, thisLoopstartPos);

                            SendURCommanderMoveJViaL(thisLoopPos, 0.4, 0.3);

                            Thread.Sleep(800);
                            while (ProgramState == 2.0)
                            {
                                Thread.Sleep(200);
                            }
                        }
                        #endregion
                    }

                    // 回传数据
                    OnSendEntireCalibrationDatas((double[,])forceRecord.Clone());
                }

                // 回到初始位置
                SendURCommanderMoveJ(InitialPosition, 0.5, 0.4);
                Thread.Sleep(800);
                while (ProgramState == 2.0)
                {
                    Thread.Sleep(200);
                }
                OnSendEntireCalibrationProcess((short)100);
            }));
        }
        #endregion

        #region 伺服模块字段
        public ServoStraightTranslation servoStraightTranslationModule; // 伺服直线运动模块
        public ServoTangentialTranslation servoTangentialTranslationModule; // 伺服切向运动模块
        public ServoTangentialTranslationWithForce servoTangentialTranslationWithForceModule; // 伺服切向运动模块(切向力驱动)
        public ServoFreeTranslation servoFreeTranslationModule; // 伺服平移运动模块
        public ServoSphereTranslation servoSphereTranslationModule; // 伺服球面运动模块
        public ServoTrackTranslation servoTrackTranslationModule; // 伺服跟踪运动模块

        public ServoTest servoTestModule; // 伺服运动测试模块
        #endregion

        #region 伺服模块方法
        /// <summary>
        /// 伺服运动所有模块的初始化
        /// </summary>
        protected void ServoMotionTotalWorkInitialization()
        {
            servoStraightTranslationModule = new ServoStraightTranslation(this, ifUse30004Port);
            servoTangentialTranslationModule = new ServoTangentialTranslation(this, ifUse30004Port);
            servoTangentialTranslationWithForceModule = new ServoTangentialTranslationWithForce(this, ifUse30004Port);
            servoFreeTranslationModule = new ServoFreeTranslation(this, ifUse30004Port);
            servoSphereTranslationModule = new ServoSphereTranslation(this, ifUse30004Port);
            servoTrackTranslationModule = new ServoTrackTranslation(this, ifUse30004Port);

            servoTestModule = new ServoTest(this, ifUse30004Port);
        }

        /// <summary>
        /// 伺服运动所有模块的工作
        /// </summary>
        protected void ServoMotionTotalWork()
        {
            servoStraightTranslationModule.ServoMotionWork(positionsTcpActual, removeGravityBaseForces);
            servoTangentialTranslationModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces);
            servoTangentialTranslationWithForceModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces);
            servoFreeTranslationModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces);
            servoSphereTranslationModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces);
            servoTrackTranslationModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces);

            servoTestModule.ServoMotionWork(positionsTcpActual, removeGravityTcpForces, speedsTcpActual);
        }
        #endregion

        #region 伺服模块外部接口方法
        /// <summary>
        /// 伺服模块中判断是否到达奇异点
        /// </summary>
        /// <returns>返回判断结果</returns>
        public bool ServoJugdeSingularReached()
        {
            // 是否到达奇异点
            if (DealWithNearSingularPoint() != 0)
            {
                ifNearSingularPoint = true;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Singular point reached.");
                return true;
            }

            return false;
        }

        /// <summary>
        /// 伺服模块开关过程对一般事务的处理
        /// </summary>
        /// <param name="ModeOpen">是否打开伺服模块</param>
        public void ServoSwitchMode(bool ModeOpen)
        {
            if (ModeOpen)
            {
                // 停止一般奇异点检测
                ifOpenSingularCheck = false;

                if (!ifUse30004Port)
                {
                    // 停止持续发送Modbus数据包
                    ifKeepSendModbusPackage = false;
                }
            }
            else
            {
                // 开始一般奇异点检测
                ifOpenSingularCheck = true;

                if (!ifUse30004Port)
                {
                    // 开始持续发送Modbus数据包
                    ifKeepSendModbusPackage = true;
                }
            }
        }

        /// <summary>
        /// 伺服模块获得当前平均力信号
        /// </summary>
        /// <returns>返回平均力信号</returns>
        public double[] ServoGetAverageForces()
        {
            double[][] getForces = ContinuousRemoveGravityTcpForces;
            int numForces = getForces.Count();
            double[] fx = new double[numForces];
            double[] fy = new double[numForces];
            double[] fz = new double[numForces];
            double[] tx = new double[numForces];
            double[] ty = new double[numForces];
            double[] tz = new double[numForces];
            for (int i = 0; i < numForces; i++)
            {
                fx[i] = getForces[i][0];
                fy[i] = getForces[i][1];
                fz[i] = getForces[i][2];
                tx[i] = getForces[i][3];
                ty[i] = getForces[i][4];
                tz[i] = getForces[i][5];
            }

            return new double[]{
                URMath.GaussAverage(fx, 4), 
                URMath.GaussAverage(fy, 4), 
                URMath.GaussAverage(fz, 4),
                URMath.GaussAverage(tx, 5),
                URMath.GaussAverage(ty, 5),
                URMath.GaussAverage(tz, 5)
            };
        }
        #endregion

        #region 非伺服模块字段
        public NonServoFindForceTranslation nonServoFindForceTranslationModule; // 非伺服寻力运动模块
        #endregion

        #region 非伺服模块方法
        /// <summary>
        /// 非伺服运动所有模块的初始化
        /// </summary>
        protected void NonServoMotionTotalWorkInitialization()
        {
            nonServoFindForceTranslationModule = new NonServoFindForceTranslation(this);
        }

        /// <summary>
        /// 非伺服运动所有模块的工作
        /// </summary>
        protected void NonServoMotionTotalWork()
        {
            nonServoFindForceTranslationModule.NonServoMotionWork(positionsTcpActual, removeGravityTcpForces);
        }

        /// <summary>
        /// 非伺服运动所有模块终止
        /// </summary>
        protected void NonServoMotionTotalAbort()
        {
            nonServoFindForceTranslationModule.NonServoMotionAbort();
        }
        #endregion
    }
}
