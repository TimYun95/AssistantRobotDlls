using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using System.Reflection;
using URCommunication;
using XMLConnection;
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
            IfEnableForceTracking = 4,
            DetectingBasicPreservedForce = 5,
            MaxAvailableRadius = 6,
            MaxAvailableAngle = 7,
            PositionOverride = 8,
            AngleOverride = 9,
            ForceOverride = 10
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

        protected double positionOverride = 0.0; // 位移倍率
        protected double angleOverride = 0.0; // 角度倍率
        protected double forceOverride = 0.0; // 力倍率

        protected bool ifEnableForceTrack = false; // 力跟踪开关

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
        protected const double detectingBasicPreservedForceLowerBound = 2.0; // 探测基准保持力大小下限
        protected const double detectingBasicPreservedForceUpperBound = 6.0; // 探测基准保持力大小上限
      
        protected const double maxAvailableRadiusLowerBound = 0.03; // 最大可行域半径下限
        protected const double maxAvailableRadiusUpperBound = 0.04; // 最大可行域半径上限
        protected const double maxAvailableAngleLowerBound = 0.7854; // 最大可旋转角度下限
        protected const double maxAvailableAngleUpperBound = 1.3090; // 最大可旋转角度上限
        
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
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { (detectingBasicPreservedForceLowerBound + 1.0).ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { maxAvailableRadiusUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { maxAvailableAngleUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { "1.00" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { "1.00" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { "1.00" });

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

            ifEnableForceTrack = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableForceTracking]);
            detectingBasicPreservedForce = double.Parse(ParameterStringList[(int)ParameterList.DetectingBasicPreservedForce]);

            maxAvailableRadius = double.Parse(ParameterStringList[(int)ParameterList.MaxAvailableRadius]);
            maxAvailableAngle = double.Parse(ParameterStringList[(int)ParameterList.MaxAvailableAngle]);

            positionOverride = double.Parse(ParameterStringList[(int)ParameterList.PositionOverride]);
            angleOverride = double.Parse(ParameterStringList[(int)ParameterList.AngleOverride]);
            forceOverride = double.Parse(ParameterStringList[(int)ParameterList.ForceOverride]);

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
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 4), new string[] { ifEnableForceTrack.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { detectingBasicPreservedForce.ToString("0.0") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { maxAvailableRadius.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { maxAvailableAngle.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { positionOverride.ToString("0.00") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { angleOverride.ToString("0.00") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { forceOverride.ToString("0.0") });

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
            
            ifEnableForceTrack = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableForceTracking)][0]);
            detectingBasicPreservedForce = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingBasicPreservedForce)][0]);

            maxAvailableRadius = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxAvailableRadius)][0]);
            maxAvailableAngle = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MaxAvailableAngle)][0]);

            positionOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.PositionOverride)][0]);
            angleOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.AngleOverride)][0]);
            forceOverride = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.ForceOverride)][0]);
            
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
            parametersList.Add(new string[] { ifEnableForceTrack.ToString() });
            parametersList.Add(new string[] { maxAvailableRadius.ToString("0.0000") });
            parametersList.Add(new string[] { maxAvailableAngle.ToString("0.0000") });
            parametersList.Add(new string[] { positionOverride.ToString("0.00") });
            parametersList.Add(new string[] { angleOverride.ToString("0.00") });
            parametersList.Add(new string[] { forceOverride.ToString("0.0") });

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
            if (ifEntireScan)
            {
                #region Entire Scan
                Task.Run(new Action(() =>
                {
                    double regionSign = ((double)ifCheckRightGalactophore) < 0.5 ? -1.0 : 1.0;

                    // 0. 移动到乳头上方抬起位置
                    double[] nippleLiftPosition = internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance, nippleTcpPostion);
                    internalProcessor.SendURCommanderMoveL(nippleLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "First go back to the position over nipple location.");

                    Thread.Sleep(800);
                    if (!JudgeIfMotionCanBeContinued()) return;
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                        if (!JudgeIfMotionCanBeContinued()) return;
                    }
                    if (!JudgeIfMotionCanBeContinued()) return;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the position over nipple location.");

                    // 1. 上半周检查
                    double angleFlag = 0.0;
                    double angleRotate = Math.PI / 2.0 * regionSign;
                    double halfSign = (int)ScanningProcess.FrontHalfRound == 0 ? -1.0 : 1.0;

                    while (angleFlag < Math.PI)
                    {
                        // 1.1 移动到抬起位置
                        double[] routeBeginLiftPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                              internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance,
                                                                              internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position over route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position over route begin location.");

                        // 1.2 移动到下压位置
                        double[] routeBeginSinkPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                                internalProcessor.MoveAlongTcpZAxis(detectingSinkDistance,
                                                                                internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginSinkPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position under route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position under route begin location.");

                        // 1.2.1 执行扫查力度校验，方向根据所测得的力定
                        Thread.Sleep(100);
                        internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                                 internalProcessor.PositionsTcpActual,
                                                                                                                                                                 checkForceSpeedLoop, checkForceAccelerationLoop,
                                                                                                                                                                 detectingBasicPreservedForce, true);

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;

                        // 1.3 单程运行参数运算
                        CalculateSingleRouteParameters(angleFlag);

                        // 1.4 开始单程运行
                        Thread.Sleep(100);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");
                        ServoTangentialTranslationWithForce.ServoDirectionAtTcp singleMovingDirection = (halfSign < 0) ? ServoTangentialTranslationWithForce.ServoDirectionAtTcp.NegativeY : ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveY;
                        internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionSetAndBegin(singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     internalProcessor.PositionsTcpActual,
                                                                                                                                                     singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     movingStopDistance,
                                                                                                                                                     detectingStopDistance,
                                                                                                                                                     0.0,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoStopMode.DistanceCondition,
                                                                                                                                                     movingSpeed,
                                                                                                                                                     detectingSpeedMax,
                                                                                                                                                     detectingSpeedMin,
                                                                                                                                                     detectingErrorForceMax,
                                                                                                                                                     detectingErrorForceMin,
                                                                                                                                                     vibratingSpeedMax,
                                                                                                                                                     (vibratingSpeedMax - vibratingSpeedMin) / (vibratingErrorForceMax - vibratingErrorForceMin),
                                                                                                                                                     inverseAngleToFriction,
                                                                                                                                                     detectingBasicPreservedForce,
                                                                                                                                                     ifAttitudeChange,
                                                                                                                                                     vibratingAttitudeMax,
                                                                                                                                                     ifEnableInitialDetectingForce,
                                                                                                                                                     ifEnableAngleCorrected);
                        if (ifEnableInitialDetectingForce) Thread.Sleep(4000);
                        else Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");

                        // 1.5 单程运行完成，进行状态检查
                        Thread.Sleep(50);
                        bool ifOpenFakeMode = false;
                        if (internalProcessor.IfNearSingularPoint)
                        {
                            internalProcessor.OpenFakeSingularPointStatus();
                            ifOpenFakeMode = true;
                        }

                        // 1.6 移动到过渡位置
                        double[] routeTransitPosition = internalProcessor.PositionsTcpActual;
                        routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[3] = routeBeginSinkPosition[3];
                        routeTransitPosition[4] = routeBeginSinkPosition[4];
                        routeTransitPosition[5] = routeBeginSinkPosition[5];
                        internalProcessor.SendURCommanderMoveL(routeTransitPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive At Transit Position.");

                        // 1.7 根据状态检查结果进行状态切换
                        if (ifOpenFakeMode)
                        {
                            internalProcessor.CloseFakeSingularPointStatus();
                        }

                        // 1.8 保存采集的数据
                        Logger.DataPrinting(internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionRecordDatas, installHanged, installTcpPosition, toolMass);

                        // 1.9 为下一程检查做准备
                        angleFlag += checkingStep;
                        angleRotate -= (regionSign * checkingStep);
                    }

                    // 2. 下半周检查
                    angleRotate += Math.PI * regionSign;
                    halfSign = (int)ScanningProcess.BehindHalfRound == 0 ? -1.0 : 1.0;

                    while (angleFlag < 2.0 * Math.PI)
                    {
                        // 2.1 移动到抬起位置
                        double[] routeBeginLiftPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                              internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance,
                                                                              internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, go to the position over route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, arrive at the position over route begin location.");

                        // 2.2 移动到下压位置
                        double[] routeBeginSinkPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                                internalProcessor.MoveAlongTcpZAxis(detectingSinkDistance,
                                                                                internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginSinkPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, go to the position under route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, arrive at the position under route begin location.");

                        // 2.2.1 执行扫查力度校验，方向根据所测得的力定
                        Thread.Sleep(100);
                        internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                                 internalProcessor.PositionsTcpActual,
                                                                                                                                                                 checkForceSpeedLoop, checkForceAccelerationLoop,
                                                                                                                                                                 detectingBasicPreservedForce, true);

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;

                        // 2.3 单程运行参数运算
                        CalculateSingleRouteParameters(angleFlag);

                        // 2.4 开始单程运行
                        Thread.Sleep(100);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");
                        ServoTangentialTranslationWithForce.ServoDirectionAtTcp singleMovingDirection = (halfSign < 0) ? ServoTangentialTranslationWithForce.ServoDirectionAtTcp.NegativeY : ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveY;
                        internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionSetAndBegin(singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     internalProcessor.PositionsTcpActual,
                                                                                                                                                     singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     movingStopDistance,
                                                                                                                                                     detectingStopDistance,
                                                                                                                                                     0.0,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoStopMode.DistanceCondition,
                                                                                                                                                     movingSpeed,
                                                                                                                                                     detectingSpeedMax,
                                                                                                                                                     detectingSpeedMin,
                                                                                                                                                     detectingErrorForceMax,
                                                                                                                                                     detectingErrorForceMin,
                                                                                                                                                     vibratingSpeedMax,
                                                                                                                                                     (vibratingSpeedMax - vibratingSpeedMin) / (vibratingErrorForceMax - vibratingErrorForceMin),
                                                                                                                                                     inverseAngleToFriction,
                                                                                                                                                     detectingBasicPreservedForce,
                                                                                                                                                     ifAttitudeChange,
                                                                                                                                                     vibratingAttitudeMax,
                                                                                                                                                     ifEnableInitialDetectingForce,
                                                                                                                                                     ifEnableAngleCorrected);
                        if (ifEnableInitialDetectingForce) Thread.Sleep(4000);
                        else Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");

                        // 2.5 单程运行完成，进行状态检查
                        Thread.Sleep(50);
                        bool ifOpenFakeMode = false;
                        if (internalProcessor.IfNearSingularPoint)
                        {
                            internalProcessor.OpenFakeSingularPointStatus();
                            ifOpenFakeMode = true;
                        }

                        // 2.6 移动到过渡位置
                        double[] routeTransitPosition = internalProcessor.PositionsTcpActual;
                        routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[3] = routeBeginSinkPosition[3];
                        routeTransitPosition[4] = routeBeginSinkPosition[4];
                        routeTransitPosition[5] = routeBeginSinkPosition[5];
                        internalProcessor.SendURCommanderMoveL(routeTransitPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive At Transit Position.");

                        // 2.7 根据状态检查结果进行状态切换
                        if (ifOpenFakeMode)
                        {
                            internalProcessor.CloseFakeSingularPointStatus();
                        }

                        // 2.8 保存采集的数据
                        Logger.DataPrinting(internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionRecordDatas, installHanged, installTcpPosition, toolMass);

                        // 2.9 为下一程检查做准备
                        angleFlag += checkingStep;
                        angleRotate -= (regionSign * checkingStep);
                    }

                    // 3. 上半周补充检查一次，保证全部循环过
                    angleRotate += Math.PI * regionSign;
                    halfSign = (int)ScanningProcess.FrontHalfRound == 0 ? -1.0 : 1.0;

                    do
                    {
                        // 3.1 移动到抬起位置
                        double[] routeBeginLiftPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                              internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance,
                                                                              internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position over route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position over route begin location.");

                        // 3.2 移动到下压位置
                        double[] routeBeginSinkPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                                internalProcessor.MoveAlongTcpZAxis(detectingSinkDistance,
                                                                                internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginSinkPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position under route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position under route begin location.");

                        // 3.2.1 执行扫查力度校验，方向根据所测得的力定
                        Thread.Sleep(100);
                        internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                                 internalProcessor.PositionsTcpActual,
                                                                                                                                                                 checkForceSpeedLoop, checkForceAccelerationLoop,
                                                                                                                                                                 detectingBasicPreservedForce, true);

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;

                        // 3.3 单程运行参数运算
                        CalculateSingleRouteParameters(angleFlag);

                        // 3.4 开始单程运行
                        Thread.Sleep(100);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");
                        ServoTangentialTranslationWithForce.ServoDirectionAtTcp singleMovingDirection = (halfSign < 0) ? ServoTangentialTranslationWithForce.ServoDirectionAtTcp.NegativeY : ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveY;
                        internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionSetAndBegin(singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     internalProcessor.PositionsTcpActual,
                                                                                                                                                     singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     movingStopDistance,
                                                                                                                                                     detectingStopDistance,
                                                                                                                                                     0.0,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoStopMode.DistanceCondition,
                                                                                                                                                     movingSpeed,
                                                                                                                                                     detectingSpeedMax,
                                                                                                                                                     detectingSpeedMin,
                                                                                                                                                     detectingErrorForceMax,
                                                                                                                                                     detectingErrorForceMin,
                                                                                                                                                     vibratingSpeedMax,
                                                                                                                                                     (vibratingSpeedMax - vibratingSpeedMin) / (vibratingErrorForceMax - vibratingErrorForceMin),
                                                                                                                                                     inverseAngleToFriction,
                                                                                                                                                     detectingBasicPreservedForce,
                                                                                                                                                     ifAttitudeChange,
                                                                                                                                                     vibratingAttitudeMax,
                                                                                                                                                     ifEnableInitialDetectingForce,
                                                                                                                                                     ifEnableAngleCorrected);
                        if (ifEnableInitialDetectingForce) Thread.Sleep(4000);
                        else Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");

                        // 3.5 单程运行完成，进行状态检查
                        Thread.Sleep(50);
                        bool ifOpenFakeMode = false;
                        if (internalProcessor.IfNearSingularPoint)
                        {
                            internalProcessor.OpenFakeSingularPointStatus();
                            ifOpenFakeMode = true;
                        }

                        // 3.6 移动到过渡位置
                        double[] routeTransitPosition = internalProcessor.PositionsTcpActual;
                        routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[3] = routeBeginSinkPosition[3];
                        routeTransitPosition[4] = routeBeginSinkPosition[4];
                        routeTransitPosition[5] = routeBeginSinkPosition[5];
                        internalProcessor.SendURCommanderMoveL(routeTransitPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive At Transit Position.");

                        // 3.7 根据状态检查结果进行状态切换
                        if (ifOpenFakeMode)
                        {
                            internalProcessor.CloseFakeSingularPointStatus();
                        }

                        // 3.8 保存采集的数据
                        Logger.DataPrinting(internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionRecordDatas, installHanged, installTcpPosition, toolMass);
                    }
                    while (false);

                    // 4. 回乳头上方
                    internalProcessor.SendURCommanderMoveL(nippleLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Finally go back to the position over nipple location.");

                    Thread.Sleep(800);
                    if (!JudgeIfMotionCanBeContinued()) return;
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                        if (!JudgeIfMotionCanBeContinued()) return;
                    }
                    if (!JudgeIfMotionCanBeContinued()) return;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the position over nipple location.");

                    // 5. 扫查结束
                    Thread.Sleep(100);
                    StopModuleNow();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End the checking process.");
                }));
                #endregion
            }
            else
            {
                #region Single Scan
                Task.Run(new Action(() =>
                {
                    double regionSign = ((double)ifCheckRightGalactophore) < 0.5 ? -1.0 : 1.0;

                    // 0. 移动到乳头上方抬起位置
                    double[] nippleLiftPosition = internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance, nippleTcpPostion);
                    internalProcessor.SendURCommanderMoveL(nippleLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "First go back to the position over nipple location.");

                    Thread.Sleep(800);
                    if (!JudgeIfMotionCanBeContinued()) return;
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                        if (!JudgeIfMotionCanBeContinued()) return;
                    }
                    if (!JudgeIfMotionCanBeContinued()) return;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the position over nipple location.");

                    // 1. 上半周检查
                    double angleFlag = angleScan - Math.Floor(angleScan / (2 * Math.PI)) * 2 * Math.PI;
                    double angleRotate = Math.PI / 2.0 * regionSign - regionSign * angleScan;
                    double halfSign = (int)ScanningProcess.FrontHalfRound == 0 ? -1.0 : 1.0;

                    if (angleFlag < Math.PI)
                    {
                        // 1.1 移动到抬起位置
                        double[] routeBeginLiftPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                              internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance,
                                                                              internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position over route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position over route begin location.");

                        // 1.2 移动到下压位置
                        double[] routeBeginSinkPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                                internalProcessor.MoveAlongTcpZAxis(detectingSinkDistance,
                                                                                internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginSinkPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, go to the position under route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Front-half check, arrive at the position under route begin location.");

                        // 1.2.1 执行扫查力度校验，方向根据所测得的力定
                        Thread.Sleep(100);
                        internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                                 internalProcessor.PositionsTcpActual,
                                                                                                                                                                 checkForceSpeedLoop, checkForceAccelerationLoop,
                                                                                                                                                                 detectingBasicPreservedForce, true);

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;

                        // 1.3 单程运行参数运算
                        CalculateSingleRouteParameters(angleFlag);

                        // 1.4 开始单程运行
                        Thread.Sleep(100);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");
                        ServoTangentialTranslationWithForce.ServoDirectionAtTcp singleMovingDirection = (halfSign < 0) ? ServoTangentialTranslationWithForce.ServoDirectionAtTcp.NegativeY : ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveY;
                        internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionSetAndBegin(singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     internalProcessor.PositionsTcpActual,
                                                                                                                                                     singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     movingStopDistance,
                                                                                                                                                     detectingStopDistance,
                                                                                                                                                     0.0,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoStopMode.DistanceCondition,
                                                                                                                                                     movingSpeed,
                                                                                                                                                     detectingSpeedMax,
                                                                                                                                                     detectingSpeedMin,
                                                                                                                                                     detectingErrorForceMax,
                                                                                                                                                     detectingErrorForceMin,
                                                                                                                                                     vibratingSpeedMax,
                                                                                                                                                     (vibratingSpeedMax - vibratingSpeedMin) / (vibratingErrorForceMax - vibratingErrorForceMin),
                                                                                                                                                     inverseAngleToFriction,
                                                                                                                                                     detectingBasicPreservedForce,
                                                                                                                                                     ifAttitudeChange,
                                                                                                                                                     vibratingAttitudeMax,
                                                                                                                                                     ifEnableInitialDetectingForce,
                                                                                                                                                     ifEnableAngleCorrected);
                        if (ifEnableInitialDetectingForce) Thread.Sleep(4000);
                        else Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");

                        // 1.5 单程运行完成，进行状态检查
                        Thread.Sleep(50);
                        bool ifOpenFakeMode = false;
                        if (internalProcessor.IfNearSingularPoint)
                        {
                            internalProcessor.OpenFakeSingularPointStatus();
                            ifOpenFakeMode = true;
                        }

                        // 1.6 移动到过渡位置
                        double[] routeTransitPosition = internalProcessor.PositionsTcpActual;
                        routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[3] = routeBeginSinkPosition[3];
                        routeTransitPosition[4] = routeBeginSinkPosition[4];
                        routeTransitPosition[5] = routeBeginSinkPosition[5];
                        internalProcessor.SendURCommanderMoveL(routeTransitPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive At Transit Position.");

                        // 1.7 根据状态检查结果进行状态切换
                        if (ifOpenFakeMode)
                        {
                            internalProcessor.CloseFakeSingularPointStatus();
                        }

                        // 1.8 保存采集的数据
                        Logger.DataPrinting(internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionRecordDatas, installHanged, installTcpPosition, toolMass);
                    }
                    else if (angleFlag <= 2.0 * Math.PI)
                    {
                        // 2. 下半周检查
                        angleRotate = 3.0 * Math.PI / 2.0 * regionSign - regionSign * angleScan;
                        halfSign = (int)ScanningProcess.BehindHalfRound == 0 ? -1.0 : 1.0;

                        // 2.1 移动到抬起位置
                        double[] routeBeginLiftPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                              internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistance,
                                                                              internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, go to the position over route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, arrive at the position over route begin location.");

                        // 2.2 移动到下压位置
                        double[] routeBeginSinkPosition = internalProcessor.MoveAlongTcpYAxis(nippleForbiddenRadius * halfSign,
                                                                                internalProcessor.MoveAlongTcpZAxis(detectingSinkDistance,
                                                                                internalProcessor.RotateByTcpZAxis(angleRotate, nippleTcpPostion)));
                        internalProcessor.SendURCommanderMoveL(routeBeginSinkPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, go to the position under route begin location.");

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Behind-half check, arrive at the position under route begin location.");

                        // 2.2.1 执行扫查力度校验，方向根据所测得的力定
                        Thread.Sleep(100);
                        internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
                                                                                                                                                                 internalProcessor.PositionsTcpActual,
                                                                                                                                                                 checkForceSpeedLoop, checkForceAccelerationLoop,
                                                                                                                                                                 detectingBasicPreservedForce, true);

                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;

                        // 2.3 单程运行参数运算
                        CalculateSingleRouteParameters(angleFlag);

                        // 2.4 开始单程运行
                        Thread.Sleep(100);
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");
                        ServoTangentialTranslationWithForce.ServoDirectionAtTcp singleMovingDirection = (halfSign < 0) ? ServoTangentialTranslationWithForce.ServoDirectionAtTcp.NegativeY : ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveY;
                        internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionSetAndBegin(singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     internalProcessor.PositionsTcpActual,
                                                                                                                                                     singleMovingDirection,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoDirectionAtTcp.PositiveZ,
                                                                                                                                                     movingStopDistance,
                                                                                                                                                     detectingStopDistance,
                                                                                                                                                     0.0,
                                                                                                                                                     ServoTangentialTranslationWithForce.ServoStopMode.DistanceCondition,
                                                                                                                                                     movingSpeed,
                                                                                                                                                     detectingSpeedMax,
                                                                                                                                                     detectingSpeedMin,
                                                                                                                                                     detectingErrorForceMax,
                                                                                                                                                     detectingErrorForceMin,
                                                                                                                                                     vibratingSpeedMax,
                                                                                                                                                     (vibratingSpeedMax - vibratingSpeedMin) / (vibratingErrorForceMax - vibratingErrorForceMin),
                                                                                                                                                     inverseAngleToFriction,
                                                                                                                                                     detectingBasicPreservedForce,
                                                                                                                                                     ifAttitudeChange,
                                                                                                                                                     vibratingAttitudeMax,
                                                                                                                                                     ifEnableInitialDetectingForce,
                                                                                                                                                     ifEnableAngleCorrected);
                        if (ifEnableInitialDetectingForce) Thread.Sleep(4000);
                        else Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End galactophore checking at route with angle of " + URMath.Rad2Deg(angleFlag).ToString("0.0") + ".");

                        // 2.5 单程运行完成，进行状态检查
                        Thread.Sleep(50);
                        bool ifOpenFakeMode = false;
                        if (internalProcessor.IfNearSingularPoint)
                        {
                            internalProcessor.OpenFakeSingularPointStatus();
                            ifOpenFakeMode = true;
                        }

                        // 2.6 移动到过渡位置
                        double[] routeTransitPosition = internalProcessor.PositionsTcpActual;
                        routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[3] = routeBeginSinkPosition[3];
                        routeTransitPosition[4] = routeBeginSinkPosition[4];
                        routeTransitPosition[5] = routeBeginSinkPosition[5];
                        internalProcessor.SendURCommanderMoveL(routeTransitPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive At Transit Position.");

                        // 2.7 根据状态检查结果进行状态切换
                        if (ifOpenFakeMode)
                        {
                            internalProcessor.CloseFakeSingularPointStatus();
                        }

                        // 2.8 保存采集的数据
                        Logger.DataPrinting(internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionRecordDatas, installHanged, installTcpPosition, toolMass);
                    }

                    // 4. 回乳头上方
                    internalProcessor.SendURCommanderMoveL(nippleLiftPosition, fastMoveAccelerationL, fastMoveSpeedL);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Finally go back to the position over nipple location.");

                    Thread.Sleep(800);
                    if (!JudgeIfMotionCanBeContinued()) return;
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                        if (!JudgeIfMotionCanBeContinued()) return;
                    }
                    if (!JudgeIfMotionCanBeContinued()) return;
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the position over nipple location.");

                    // 5. 扫查结束
                    Thread.Sleep(100);
                    StopModuleNow();
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End the checking process.");
                }));
                #endregion
            }
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
            internalProcessor.servoFreeTranslationModule.ServoMotionAbort();
            internalProcessor.servoTangentialTranslationWithForceModule.ServoMotionAbort();
        }

        #endregion




    }
}
