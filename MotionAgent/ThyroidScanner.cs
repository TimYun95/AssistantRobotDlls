using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using System.Reflection;
using URCommunication;
using XMLConnection;
using MathFunction;
using URServo;
using URNonServo;
using LogPrinter;

namespace URModule
{
    /// <summary>
    /// 甲状腺扫描模块类
    /// </summary>
    public class ThyroidScanner : OperateModuleBase
    {
        #region 枚举
        /// <summary>
        /// 参数列表
        /// </summary>
        public enum ParameterList : int
        {
            DetectingErrorForceMin = 0,
            DetectingErrorForceMax = 1,
            DetectingSpeedMin = 2,
            DetectingSpeedMax = 3,
            IfEnableForceKeeping = 4,
            IfEnableForceTracking = 5,
            DetectingBasicPreservedForce = 6,
            MaxAvailableRadius = 7,
            MaxAvailableAngle = 8,
            StopRadius = 9,
            MaxDistPeriod = 10,
            MaxAnglePeriod = 11,
            PositionOverride = 12,
            AngleOverride = 13,
            ForceOverride = 14,
            IfEnableAttitudeTracking = 15,
            IfEnableTranslationTracking = 16
        }
        #endregion

        #region 字段
        protected const string xmlFileName = "ThyroidScanning.xml"; // XML文件名
        protected const string replaceFilePath = "ThyroidScanning\\"; // XML文件转存目录

        protected bool ifStartScanPositionFound = false; // 是否找到了甲状腺扫描起始位置

        protected const double checkForceAcceleration = 0.005;  // 扫查力度校验移动加速度
        protected const double checkForceSpeed = 0.0005; // 扫查力度校验移动速度

        public delegate void SendStringList(List<string[]> ParametersList); // string[]列表发送委托
        /// <summary>
        /// 发送当前模块参数
        /// </summary>
        public event SendStringList OnSendModuleParameters;

        private static readonly object controlParamLocker = new object(); // 部分控制参数锁
        #endregion

        #region 基本控制字段
        protected double detectingErrorForceMin = 0.0; // 探测方向误差力最小值
        protected double detectingErrorForceMax = 0.0; // 探测方向误差力最大值
        protected double detectingSpeedMin = 0.0; // 探测方向运动速度最小值
        protected double detectingSpeedMax = 0.0; // 探测方向运动速度最大值

        protected double detectingBasicPreservedForce = 0.0; // 探测基准保持力大小
        #endregion

        #region 高级控制字段
        protected double maxAvailableRadius = 0.0; // 最大可行域半径
        protected double maxAvailableAngle = 0.0; // 最大可旋转角度

        protected double stopRadius = 0.0; // 终止可行域半径
        protected double maxDistPeriod = 0.0; // 单周期最大移动距离
        protected double maxAnglePeriod = 0.0; // 单周期最大旋转角度

        protected double positionOverride = 0.0; // 位移倍率
        protected double angleOverride = 0.0; // 角度倍率
        protected double forceOverride = 0.0; // 力倍率

        protected bool ifEnableForceTrack = false; // 力跟踪开关
        protected bool ifEnableForceKeep = false; // 力保持开关
        protected bool ifEnableAttitudeTrack = false; // 姿态跟踪开关
        protected bool ifEnableTranslationTrack = false; // 位移跟踪开关

        protected double[] startTcpPostion = new double[6]; // 起始位置对应的Tcp位置
        #endregion

        #region 配置参数校验字段
        protected const double detectingErrorForceMinLowerBound = 0.0; // 探测方向误差力最小值下限
        protected const double detectingErrorForceMinUpperBound = 1.0; // 探测方向误差力最小值上限
        protected const double detectingErrorForceMaxLowerBound = 1.5; // 探测方向误差力最大值下限
        protected const double detectingErrorForceMaxUpperBound = 3.0; // 探测方向误差力最大值上限
        protected const double detectingSpeedMinLowerBound = 0.0000; // 探测方向运动速度最小值下限
        protected const double detectingSpeedMinUpperBound = 0.0003; // 探测方向运动速度最小值上限

        protected const double detectingSpeedMaxLowerBound = 0.0002; // 探测方向运动速度最大值下限
        protected const double detectingSpeedMaxUpperBound = 0.0008; // 探测方向运动速度最大值上限
        protected const double detectingBasicPreservedForceLowerBound = 3.0; // 探测基准保持力大小下限
        protected const double detectingBasicPreservedForceUpperBound = 6.0; // 探测基准保持力大小上限

        protected const double maxAvailableRadiusLowerBound = 0.3; // 最大可行域半径下限
        protected const double maxAvailableRadiusUpperBound = 0.4; // 最大可行域半径上限
        protected const double maxAvailableAngleLowerBound = 0.7854; // 最大可旋转角度下限
        protected const double maxAvailableAngleUpperBound = 1.3090; // 最大可旋转角度上限

        protected const double stopRadiusLowerBound = 0.4; // 终止可行域半径下限
        protected const double stopRadiusUpperBound = 0.5; // 终止可行域半径上限
        protected const double maxDistPeriodLowerBound = 0.0002; // 单周期最大移动距离下限
        protected const double maxDistPeriodUpperBound = 0.0009; // 单周期最大移动距离上限
        protected const double maxAnglePeriodLowerBound = 0.0005; // 单周期最大旋转角度下限
        protected const double maxAnglePeriodUpperBound = 0.0030; // 单周期最大旋转角度上限

        protected const double positionOverrideLowerBound = 0.25; // 位移倍率下限
        protected const double positionOverrideUpperBound = 1.25; // 位移倍率上限
        protected const double angleOverrideLowerBound = 0.25; // 角度倍率下限
        protected const double angleOverrideUpperBound = 1.25; // 角度倍率上限
        protected const double forceOverrideLowerBound = 0.5; // 力倍率下限
        protected const double forceOverrideUpperBound = 2.0; // 力倍率上限
        #endregion

        #region 属性
        /// <summary>
        /// 是否找到了扫描起始位置
        /// </summary>
        public bool IfStartScanPositionFound
        {
            get { return ifStartScanPositionFound; }
        }

        /// <summary>
        /// 是否打开力跟踪
        /// </summary>
        public bool IfEnableForceTrack
        {
            get { return ifEnableForceTrack; }
            set { ifEnableForceTrack = value; }
        }

        /// <summary>
        /// 是否打开力保持
        /// </summary>
        public bool IfEnableForceKeep
        {
            get { return ifEnableForceKeep; }
            set { ifEnableForceKeep = value; }
        }

        /// <summary>
        /// 是否打开姿态跟踪
        /// </summary>
        public bool IfEnableAttitudeTrack
        {
            get { return ifEnableAttitudeTrack; }
            set { ifEnableAttitudeTrack = value; }
        }

        /// <summary>
        /// 是否打开位移跟踪
        /// </summary>
        public bool IfEnableTranslationTrack
        {
            get { return ifEnableTranslationTrack; }
            set { ifEnableTranslationTrack = value; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="InternalProcessor">内部UR数据处理类，用以获得相关数据交换和控制权</param>
        /// <param name="RecordedJointAngles">数据库中记录的当前工具的初始关节角度</param>
        /// <param name="InstallTcpPosition">数据库中记录的当前工具的安装TCP位置</param>
        /// <param name="InstallHanged">数据库中记录的当前工具的安装方式</param>
        /// <param name="ToolMass">数据库中记录的当前工具的重力</param>
        public ThyroidScanner(URDataProcessor InternalProcessor, double[] RecordedJointAngles, double[] InstallTcpPosition, bool InstallHanged, double ToolMass) :
            base(InternalProcessor, RecordedJointAngles, InstallTcpPosition, InstallHanged, ToolMass)
        {
            InitialXmlProcessor(); // 初始化XML处理者
        }

        /// <summary>
        /// 初始化XML文件处理者
        /// </summary>
        protected override void InitialXmlProcessor()
        {
            Dictionary<string, string[]> parametersDictionary = new Dictionary<string, string[]>(100);
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 0), new string[] { detectingErrorForceMinLowerBound.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 1), new string[] { (detectingErrorForceMaxLowerBound + 0.5).ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 2), new string[] { detectingSpeedMinLowerBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 3), new string[] { (detectingSpeedMaxLowerBound + 0.0003).ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 4), new string[] { "True" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { "True" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { detectingBasicPreservedForceLowerBound.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { maxAvailableRadiusUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { maxAvailableAngleUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { stopRadiusUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { maxDistPeriodLowerBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 11), new string[] { maxAnglePeriodLowerBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 12), new string[] { "1.00" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 13), new string[] { "1.00" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 14), new string[] { "1.00" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 15), new string[] { "True" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 16), new string[] { "True" });

            xmlProcessor = new XMLConnector(xmlFileName, replaceFilePath, parametersDictionary);
        }

        /// <summary>
        /// 从string列表中获得并保存到模块参数
        /// </summary>
        /// <param name="ParameterStringList">参数string列表</param>
        protected override void GetParametersFromString(List<string> ParameterStringList)
        {
            detectingErrorForceMin = double.Parse(ParameterStringList[(int)ParameterList.DetectingErrorForceMin]);
            detectingErrorForceMax = double.Parse(ParameterStringList[(int)ParameterList.DetectingErrorForceMax]);
            detectingSpeedMin = double.Parse(ParameterStringList[(int)ParameterList.DetectingSpeedMin]);
            detectingSpeedMax = double.Parse(ParameterStringList[(int)ParameterList.DetectingSpeedMax]);

            ifEnableForceKeep = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableForceKeeping]);
            ifEnableForceTrack = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableForceTracking]);
            detectingBasicPreservedForce = double.Parse(ParameterStringList[(int)ParameterList.DetectingBasicPreservedForce]);

            maxAvailableRadius = double.Parse(ParameterStringList[(int)ParameterList.MaxAvailableRadius]);
            maxAvailableAngle = double.Parse(ParameterStringList[(int)ParameterList.MaxAvailableAngle]);

            stopRadius = double.Parse(ParameterStringList[(int)ParameterList.StopRadius]);
            maxDistPeriod = double.Parse(ParameterStringList[(int)ParameterList.MaxDistPeriod]);
            maxAnglePeriod = double.Parse(ParameterStringList[(int)ParameterList.MaxAnglePeriod]);

            positionOverride = double.Parse(ParameterStringList[(int)ParameterList.PositionOverride]);
            angleOverride = double.Parse(ParameterStringList[(int)ParameterList.AngleOverride]);
            forceOverride = double.Parse(ParameterStringList[(int)ParameterList.ForceOverride]);

            ifEnableAttitudeTrack = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableAttitudeTracking]);
            ifEnableTranslationTrack = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableTranslationTracking]);

            CalculateAndCheckParametersBothExposedAndHidden(); // 加载后限制部分参数并计算相关参数
        }

        /// <summary>
        /// 将模块参数保存到XML文件中
        /// </summary>
        protected override void SaveParametersToXml()
        {
            Dictionary<string, string[]> parametersDictionary = new Dictionary<string, string[]>(100);
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 0), new string[] { detectingErrorForceMin.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 1), new string[] { detectingErrorForceMax.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 2), new string[] { detectingSpeedMin.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 3), new string[] { detectingSpeedMax.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 4), new string[] { ifEnableForceKeep.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { ifEnableForceTrack.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { detectingBasicPreservedForce.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { maxAvailableRadius.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { maxAvailableAngle.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { stopRadius.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { maxDistPeriod.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 11), new string[] { maxAnglePeriod.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 12), new string[] { positionOverride.ToString("0.00") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 13), new string[] { angleOverride.ToString("0.00") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 14), new string[] { forceOverride.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 15), new string[] { ifEnableAttitudeTrack.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 16), new string[] { ifEnableTranslationTrack.ToString() });

            xmlProcessor.SaveXML(parametersDictionary);
        }

        /// <summary>
        /// 从XML文件中加载到模块参数
        /// </summary>
        protected override void LoadParametersFromXml()
        {
            Dictionary<string, string[]> parametersDictionary = xmlProcessor.ReadXml();
            detectingErrorForceMin = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingErrorForceMin)][0]);
            detectingErrorForceMax = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingErrorForceMax)][0]);
            detectingSpeedMin = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingSpeedMin)][0]);
            detectingSpeedMax = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingSpeedMax)][0]);

            ifEnableForceKeep = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableForceKeeping)][0]);
            ifEnableForceTrack = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableForceTracking)][0]);
            detectingBasicPreservedForce = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingBasicPreservedForce)][0]);

            maxAvailableRadius = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxAvailableRadius)][0]);
            maxAvailableAngle = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxAvailableAngle)][0]);

            stopRadius = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.StopRadius)][0]);
            maxDistPeriod = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxDistPeriod)][0]);
            maxAnglePeriod = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxAnglePeriod)][0]);

            positionOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.PositionOverride)][0]);
            angleOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.AngleOverride)][0]);
            forceOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.ForceOverride)][0]);

            ifEnableAttitudeTrack = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableAttitudeTracking)][0]);
            ifEnableTranslationTrack = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableTranslationTracking)][0]);

            CalculateAndCheckParametersBothExposedAndHidden(); // 加载后限制部分参数并计算相关参数
        }

        /// <summary>
        /// 计算并检查所有相关配置参数
        /// </summary>
        protected override void CalculateAndCheckParametersBothExposedAndHidden()
        {
            CheckParametersWithLimitations(); // 限制
        }

        /// <summary>
        /// 将模块参数抛出
        /// </summary>
        protected override void OutputParameters()
        {
            List<string[]> parametersList = new List<string[]>(100);
            parametersList.Add(new string[] { detectingErrorForceMin.ToString("0.0") });
            parametersList.Add(new string[] { detectingErrorForceMax.ToString("0.0") });
            parametersList.Add(new string[] { detectingSpeedMin.ToString("0.0000") });
            parametersList.Add(new string[] { detectingSpeedMax.ToString("0.0000") });
            parametersList.Add(new string[] { ifEnableForceKeep.ToString() });
            parametersList.Add(new string[] { ifEnableForceTrack.ToString() });
            parametersList.Add(new string[] { detectingBasicPreservedForce.ToString("0.0") });

            parametersList.Add(new string[] { maxAvailableRadius.ToString("0.0000") });
            parametersList.Add(new string[] { maxAvailableAngle.ToString("0.0000") });
            parametersList.Add(new string[] { stopRadius.ToString("0.0000") });
            parametersList.Add(new string[] { maxDistPeriod.ToString("0.0000") });
            parametersList.Add(new string[] { maxAnglePeriod.ToString("0.0000") });
           
            parametersList.Add(new string[] { positionOverride.ToString("0.00") });
            parametersList.Add(new string[] { angleOverride.ToString("0.00") });
            parametersList.Add(new string[] { forceOverride.ToString("0.0") });
            parametersList.Add(new string[] { ifEnableAttitudeTrack.ToString() });
            parametersList.Add(new string[] { ifEnableTranslationTrack.ToString() });

            OnSendModuleParameters(parametersList);
        }

        /// <summary>
        /// 通过极限限制部分参数
        /// </summary>
        protected void CheckParametersWithLimitations()
        {
            if (detectingErrorForceMin < detectingErrorForceMinLowerBound) detectingErrorForceMin = detectingErrorForceMinLowerBound;
            if (detectingErrorForceMin > detectingErrorForceMinUpperBound) detectingErrorForceMin = detectingErrorForceMinUpperBound;
            if (detectingErrorForceMax < detectingErrorForceMaxLowerBound) detectingErrorForceMax = detectingErrorForceMaxLowerBound;
            if (detectingErrorForceMax > detectingErrorForceMaxUpperBound) detectingErrorForceMax = detectingErrorForceMaxUpperBound;

            if (detectingSpeedMin < detectingSpeedMinLowerBound) detectingSpeedMin = detectingSpeedMinLowerBound;
            if (detectingSpeedMin > detectingSpeedMinUpperBound) detectingSpeedMin = detectingSpeedMinUpperBound;
            if (detectingSpeedMax < detectingSpeedMaxLowerBound) detectingSpeedMax = detectingSpeedMaxLowerBound;
            if (detectingSpeedMax > detectingSpeedMaxUpperBound) detectingSpeedMax = detectingSpeedMaxUpperBound;

            if (detectingBasicPreservedForce < detectingBasicPreservedForceLowerBound) detectingBasicPreservedForce = detectingBasicPreservedForceLowerBound;
            if (detectingBasicPreservedForce > detectingBasicPreservedForceUpperBound) detectingBasicPreservedForce = detectingBasicPreservedForceUpperBound;

            if (maxAvailableRadius < maxAvailableRadiusLowerBound) maxAvailableRadius = maxAvailableRadiusLowerBound;
            if (maxAvailableRadius > maxAvailableRadiusUpperBound) maxAvailableRadius = maxAvailableRadiusUpperBound;
            if (maxAvailableAngle < maxAvailableAngleLowerBound) maxAvailableAngle = maxAvailableAngleLowerBound;
            if (maxAvailableAngle > maxAvailableAngleUpperBound) maxAvailableAngle = maxAvailableAngleUpperBound;

            if (stopRadius < stopRadiusLowerBound) stopRadius = stopRadiusLowerBound;
            if (stopRadius > stopRadiusUpperBound) stopRadius = stopRadiusUpperBound;
            if (maxDistPeriod < maxDistPeriodLowerBound) maxDistPeriod = maxDistPeriodLowerBound;
            if (maxDistPeriod > maxDistPeriodUpperBound) maxDistPeriod = maxDistPeriodUpperBound;
            if (maxAnglePeriod < maxAnglePeriodLowerBound) maxAnglePeriod = maxAnglePeriodLowerBound;
            if (maxAnglePeriod > maxAnglePeriodUpperBound) maxAnglePeriod = maxAnglePeriodUpperBound;

            if (positionOverride < positionOverrideLowerBound) positionOverride = positionOverrideLowerBound;
            if (positionOverride > positionOverrideUpperBound) positionOverride = positionOverrideUpperBound;
            if (angleOverride < angleOverrideLowerBound) angleOverride = angleOverrideLowerBound;
            if (angleOverride > angleOverrideUpperBound) angleOverride = angleOverrideUpperBound;
            if (forceOverride < forceOverrideLowerBound) forceOverride = forceOverrideLowerBound;
            if (forceOverride > forceOverrideUpperBound) forceOverride = forceOverrideUpperBound;
        }

        /// <summary>
        /// 准备开始运行模块
        /// </summary>
        protected override void AttemptToStartModule()
        {
            // 0. 基类函数调用
            base.AttemptToStartModule();

            // 1. 加载XML文件并外推配置参数
            LoadParametersFromXmlAndOutput();

            // 2. 部分参数还原
            ifStartScanPositionFound = false;
        }

        /// <summary>
        /// 寻找扫描起始位置
        /// </summary>
        public void FindScanningStartTcpPosition()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.SendURCommanderBeginTeachMode();

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin to find scanning begin position.");
                }));
            }
        }

        /// <summary>
        /// 确认找到了扫描起始位置
        /// </summary>
        public void ConfirmScanningStartTcpPositionFound()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.SendURCommanderEndTeachMode();

                    Thread.Sleep(200);
                    startTcpPostion = internalProcessor.PositionsTcpActual;
                    ifStartScanPositionFound = true;

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Scanning start position is found and recorded.");
                }));
            }
        }

        /// <summary>
        /// 模块执行的工作
        /// </summary>
        protected override void ModuleWork()
        {
            Task.Run(new Action(() =>
            {
                // 0. 移动到开始扫描位置
                internalProcessor.SendURCommanderMoveL(startTcpPostion, fastMoveAccelerationL, fastMoveSpeedL);
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "First go to the start position.");
                Thread.Sleep(800);
                if (!JudgeIfMotionCanBeContinued()) return;
                while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                {
                    Thread.Sleep(200);
                    if (!JudgeIfMotionCanBeContinued()) return;
                }
                if (!JudgeIfMotionCanBeContinued()) return;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the start position.");

                // 1. 执行扫查力度校验
                Thread.Sleep(100);
                internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                         internalProcessor.PositionsTcpActual,
                                                                                                                                                         checkForceSpeed, checkForceAcceleration,
                                                                                                                                                         detectingBasicPreservedForce, true);

                Thread.Sleep(800);
                if (!JudgeIfMotionCanBeContinued()) return;
                while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                {
                    Thread.Sleep(200);
                    if (!JudgeIfMotionCanBeContinued()) return;
                }
                if (!JudgeIfMotionCanBeContinued()) return;

                // 2. 设置初始目标
                RefreshAimPostion(0, new double[] { 0.0, 0.0, startTcpPostion[3], startTcpPostion[4], startTcpPostion[5], -detectingBasicPreservedForce });
                Thread.Sleep(100);

                // 3. 开始运行
                Thread.Sleep(100);
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin checking.");
                internalProcessor.servoTrackTranslationModule.ServoMotionSetAndBegin(stopRadius,
                                                                                                                                      maxDistPeriod,
                                                                                                                                      maxAnglePeriod,
                                                                                                                                      ifEnableForceKeep,
                                                                                                                                      detectingSpeedMax,
                                                                                                                                      detectingSpeedMin,
                                                                                                                                      detectingErrorForceMax,
                                                                                                                                      detectingErrorForceMin);
                Thread.Sleep(800);
                if (!JudgeIfMotionCanBeContinued()) return;
                while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                {
                    Thread.Sleep(200);
                    if (!JudgeIfMotionCanBeContinued()) return;
                }
                if (!JudgeIfMotionCanBeContinued()) return;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End checking.");

                // 4. 运行完成
                Thread.Sleep(50);
                internalProcessor.SendURCommanderMoveLViaJ(initialJointAngles, normalMoveAccelerationL, normalMoveSpeedL);
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Go back to initial joint position.");

                Thread.Sleep(800);
                if (!JudgeIfMotionCanBeContinued()) return;
                while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                {
                    Thread.Sleep(200);
                    if (!JudgeIfMotionCanBeContinued()) return;
                }
                if (!JudgeIfMotionCanBeContinued()) return;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at initial joint position.");

                // 5. 扫查结束
                Thread.Sleep(100);
                StopModuleNow();
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End the checking process.");
            }));
        }

        /// <summary>
        /// 停止模块执行工作中的运动
        /// </summary>
        protected override void StopModuleWork()
        {
            // 转存XML文件
            if (ifAutoReplaceConfiguration)
            {
                AutoReplaceXml();
            }
        }

        /// <summary>
        /// 立刻停止模块运行
        /// </summary>
        protected override void StopAllMotionInModule()
        {
            // 1. 急停模块
            base.StopAllMotionInModule();

            // 2. 模块正常结束
            StopModuleWork();
        }

        /// <summary>
        /// 停止所涉及的模块
        /// </summary>
        protected override void StopRelevantModule()
        {
            internalProcessor.nonServoFindForceTranslationModule.NonServoMotionAbort();
            internalProcessor.servoTrackTranslationModule.ServoMotionAbort();
        }

        /// <summary>
        /// 更新控制目标参考值
        /// </summary>
        /// <param name="flag">运动标号</param>
        /// <param name="values">运动参考值</param>
        public void RefreshAimPostion(int flag, double[] values)
        {
            if (workingStatus == WorkStatus.WorkRunning)
            {
                double posFactor, attFactor, fosFactor;
                bool posEnable, attEnable, fosKeepEnable, fosTrackEnable;
                lock (controlParamLocker)
                {
                    posFactor = positionOverride;
                    attFactor = angleOverride;
                    fosFactor = forceOverride;

                    posEnable = ifEnableTranslationTrack;
                    attEnable = ifEnableAttitudeTrack;
                    fosKeepEnable = ifEnableForceKeep;
                    fosTrackEnable = ifEnableForceTrack;
                }

                int nextFlag = flag;
                double xDist = values[0] * posFactor;
                double yDist = values[1] * posFactor;
                double[] posture = new double[] { values[2] * attFactor, 
                                                                       values[3] * attFactor, 
                                                                       values[4] * attFactor };
                double force = values[5] * fosFactor;

                // 1. 检验距离是否太大
                if (URMath.LengthOfArray(new double[] { xDist, yDist, 0 }) > maxAvailableRadius)
                {
                    double theta = Math.Atan2(yDist, xDist);
                    xDist = maxAvailableRadius * Math.Cos(theta);
                    yDist = maxAvailableRadius * Math.Sin(theta);
                }

                // 2. 检测转角是否过大
                double recAngle = URMath.LengthOfArray(posture);
                if (recAngle > maxAvailableAngle)
                {
                    posture[0] = posture[0] / recAngle * maxAvailableAngle;
                    posture[1] = posture[1] / recAngle * maxAvailableAngle;
                    posture[2] = posture[2] / recAngle * maxAvailableAngle;
                }

                // 3. 转换到当前的目标姿态
                Quatnum initialQ = URMath.AxisAngle2Quatnum(
                     new double[] { startTcpPostion[3], startTcpPostion[4], startTcpPostion[5] });
                double rotateAngle = URMath.LengthOfArray(posture);
                double[] rotateAxisAtBase =
                    URMath.FindDirectionToSecondReferenceFromFirstReference
                    (new double[] { posture[0] / rotateAngle, posture[1] / rotateAngle, posture[2] / rotateAngle },
                     URMath.InvQuatnum(initialQ));
                Quatnum rotateQ = URMath.AxisAngle2Quatnum(new double[] { rotateAxisAtBase[0] * rotateAngle, rotateAxisAtBase[1] * rotateAngle, rotateAxisAtBase[2] * rotateAngle });
                double[] aimAttitude = URMath.Quatnum2AxisAngle(
                    URMath.QuatnumRotate(new Quatnum[] { initialQ, rotateQ }));

                // 4. 检测目标姿态是否过限
                double[] aimZDirection =
                    URMath.FindDirectionToSecondReferenceFromFirstReference(
                        new double[] { 0.0, 0.0, 1.0 },
                        URMath.AxisAngle2Quatnum(aimAttitude));
                if (URMath.VectorDotMultiply(
                    aimZDirection, new double[] { 0.0, 0.0, 1.0 }) > 0)
                {
                    double aimAttitudeAngle = URMath.LengthOfArray(aimAttitude);
                    double[] aimAttitudeAxis = new double[] {
                        aimAttitude[0] / aimAttitudeAngle, 
                        aimAttitude[1] / aimAttitudeAngle, 
                        aimAttitude[2] / aimAttitudeAngle};

                    double newSin = 0.0;
                    if (Math.Pow(aimAttitudeAxis[2], 2) > 0.5)
                        newSin = 1.0;
                    else
                        newSin = Math.Sqrt(0.5 / (1 - Math.Pow(aimAttitudeAxis[2], 2)));
                    if (Math.Abs(newSin) > 1.0) newSin = Math.Sign(newSin) * 1.0;
                    double newAngle = 2.0 * Math.Asin(newSin);
                    if (newAngle == System.Double.NaN) newAngle = Math.Sign(newSin) * Math.PI;
                    aimAttitude[0] = aimAttitudeAxis[0] * newAngle;
                    aimAttitude[1] = aimAttitudeAxis[1] * newAngle;
                    aimAttitude[2] = aimAttitudeAxis[2] * newAngle;
                }

                // 5. 将参数输入模块
                // 输入模块的参数为：
                // 相对起始扫描点的两维坐标，
                // 相对基座原点的姿态，
                // 接触压力
                internalProcessor.servoTrackTranslationModule.ServoMotionRefreshRefValues(
                    nextFlag, new double?[] { 
                        posEnable ? new double?(xDist) : null, 
                        posEnable ? new double?(yDist) : null, 
                        attEnable ? new double?(aimAttitude[0]) : null, 
                        attEnable ? new double?(aimAttitude[1]) : null, 
                        attEnable ? new double?(aimAttitude[2]) : null, 
                        fosTrackEnable ? force : detectingBasicPreservedForce});

                // 6. 力保持使能更改
                internalProcessor.servoTrackTranslationModule.ServoMotionForceTrackEnable = fosKeepEnable;
            }
        }

        /// <summary>
        /// 在模块工作中更新部分控制参数
        /// </summary>
        /// <param name="dataList">部分控制参数列表</param>
        public void RefreshPartParameters(List<string> dataList)
        {
            if (workingStatus != WorkStatus.WorkRunning) return;

            double posFactor = double.Parse(dataList[0]);
            double attFactor = double.Parse(dataList[1]);
            double fosFactor = double.Parse(dataList[2]);
            bool posEnable = bool.Parse(dataList[3]);
            bool attEnable = bool.Parse(dataList[4]);
            bool fosKeepEnable = bool.Parse(dataList[5]);
            bool fosTrackEnable = bool.Parse(dataList[6]);

            lock (controlParamLocker)
            {
                positionOverride = posFactor;
                angleOverride = attFactor;
                forceOverride = fosFactor;

                ifEnableTranslationTrack = posEnable;
                ifEnableAttitudeTrack = attEnable;
                ifEnableForceKeep = fosKeepEnable;
                ifEnableForceTrack = fosTrackEnable;
            }

            Task.Run(new Action(() =>
            {
                Dictionary<string, string[]> parametersDictionary = xmlProcessor.ReadXml();
                parametersDictionary.ElementAt(4).Value[0] = dataList[5];
                parametersDictionary.ElementAt(5).Value[0] = dataList[6];
                parametersDictionary.ElementAt(12).Value[0] = dataList[0];
                parametersDictionary.ElementAt(13).Value[0] = dataList[1];
                parametersDictionary.ElementAt(14).Value[0] = dataList[2];
                parametersDictionary.ElementAt(15).Value[0] = dataList[4];
                parametersDictionary.ElementAt(16).Value[0] = dataList[3];

                xmlProcessor.SaveXML(parametersDictionary);
            }));
        }






        #endregion
    }
}
