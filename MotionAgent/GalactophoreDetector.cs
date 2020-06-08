using System;
using System.Collections.Generic;
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
    /// 乳腺检查模块类
    /// </summary>
    public class GalactophoreDetector : OperateModuleBase
    {
        #region 枚举
        /// <summary>
        /// 摆动幅度
        /// </summary>
        public enum VibratingMagnitude : byte
        {
            Small = 0,
            Medium,
            Large
        }

        /// <summary>
        /// 探头移动速度
        /// </summary>
        public enum MovingLevel : byte
        {
            Slow = 0,
            Medium,
            Fast
        }

        /// <summary>
        /// 探测力度
        /// </summary>
        public enum DetectingIntensity : byte
        {
            Light = 0,
            SlightlyLight,
            SlightltHeavy,
            Heavy
        }

        /// <summary>
        /// 贴合程度
        /// </summary>
        public enum AligningDegree : byte
        {
            Loose = 0,
            Tight
        }

        /// <summary>
        /// 参数列表
        /// </summary>
        public enum ParameterList : int
        {
            DetectingErrorForceMin = 0,
            DetectingErrorForceMax = 1,
            DetectingSpeedMin = 2,
            IfEnableAngleCorrected = 3,
            VibratingAttitudeMaxAtSmoothPart = 4,
            VibratingAttitudeMinAtSteepPart = 5,
            VibratingAttitudeMaxAtSteepPart = 6,
            NippleForbiddenRadius = 7,
            MovingStopDistance = 8,
            DetectingStopDistance = 9,
            DetectingSafetyLiftDistance = 10,
            IfEnableInitialDetectingForce = 11,
            DetectingSinkDistance = 12,
            VibratingAngleDegree = 13,
            MovingSpeedDegree = 14,
            DetectingForceDegree = 15,
            DetectingAlignDegree = 16,
            MovingUpEdgeDistance = 17,
            MovingLeftEdgeDistance = 18,
            MovingDownEdgeDistance = 19,
            MovingRightEdgeDistance = 20,
            IfAutoReplaceConfiguration = 21,
            IfCheckRightGalactophore = 22,
            IdentifyBoundaryMode = 23,
            CheckingStep = 24
        }

        /// <summary>
        /// 扫查半周数
        /// </summary>
        protected enum ScanningProcess : int
        {
            FrontHalfRound = 0,
            BehindHalfRound = 1
        }

        /// <summary>
        /// 扫查部位
        /// </summary>
        public enum ScanningRegion : int
        {
            LeftGalactophore = 0,
            RightGalactophore = 1
        }

        /// <summary>
        /// 确定边界的方法
        /// </summary>
        public enum IdentifyBoundary : byte
        {
            OnlyUpBoundary = 0,
            UpDownBoundary = 1,
            AllBoundary = 2
        }

        #endregion

        #region 字段
        protected const string xmlFileName = "GalactophoreDetection.xml"; // XML文件名
        protected const string replaceFilePath = "GalactophoreDetection\\"; // XML文件转存目录

        protected const double findConfigurationParameterSpeedMin = 0.0001; // 配置参数搜寻最低速度
        protected const double findConfigurationParameterSpeedMax = 0.0012; // 配置参数搜寻最高速度
        protected const double findConfigurationParameterForceMin = 1.5; // 配置参数搜寻最低力
        protected const double findConfigurationParameterForceMax = 5.0; // 配置参数搜寻最高力
        protected const double findConfigurationParameterDistanceMax = 0.5; // 配置参数搜寻最远距离

        protected bool ifNipplePositionFound = false; // 是否找到了乳头位置

        protected const double checkForceAcceleration = 0.01;  // 扫查力度校验移动加速度
        protected const double checkForceSpeed = 0.0015; // 扫查力度校验移动速度
        protected const double checkForceAccelerationLoop = 0.005;  // 每轮扫查力度校验移动加速度
        protected const double checkForceSpeedLoop = 0.0005; // 每轮扫查力度校验移动速度

        protected double[] cubicSplineCurveConstantTerm = new double[8]; // 三次样条插值参考边界各段常数项
        protected double[] cubicSplineCurveLinearTermCoefficient = new double[8]; // 三次样条插值参考边界各段线性项系数
        protected double[] cubicSplineCurveQuadraticTermCoefficient = new double[8]; // 三次样条插值参考边界各段二次项系数
        protected double[] cubicSplineCurveCubicTermCoefficient = new double[8]; // 三次样条插值参考边界各段三次项系数

        protected const double steepPartHalfAngle = Math.PI / 4.0; // 陡峭段半范围角度
        protected const double smoothPartHalfAngle = Math.PI / 2.0; // 平滑段半范围角度

        public const double notChange = -1.0; // 配置参数不作更改时给的值
        public delegate void SendStringList(List<string[]> ParametersList); // string[]列表发送委托
        /// <summary>
        /// 发送当前模块参数
        /// </summary>
        public event SendStringList OnSendModuleParameters;

        protected bool ifEntireScan = true;
        protected double angleScan = 0.0;
        protected bool ifAttitudeChange = true;
        #endregion

        #region 基本控制字段
        protected double detectingErrorForceMin = 0.0; // 探测方向误差力最小值
        protected double detectingErrorForceMax = 0.0; // 探测方向误差力最大值
        protected double detectingSpeedMin = 0.0; // 探测方向运动速度最小值

        protected double detectingSpeedMax = 0.0; // 探测方向运动速度最大值

        protected const double vibratingErrorForceMin = 0.0; // 摆动方向误差力最小值
        protected const double vibratingErrorForceMax = 1.0; // 摆动方向误差力最大值
        protected const double vibratingSpeedMin = 0.0; // 摆动方向运动速度最小值
        protected const double inverseAngleToFriction = 20.0; // 摆动方向角度误差转换到模板力误差的系数

        protected double vibratingSpeedMax = 0.0; // 摆动方向运动速度最大值
        //protected int vibratingAttitudeJudgeSamplingNumber = 0; // 姿态判别所需的采样个数
        //protected int vibratingAttitudeJudgeDifferenceInterval = 0; // 姿态判别所需的差分距离
        //protected int vibratingAttitudeJudgeExtensionPeriod = 0; // 姿态判别所需的外延周期
        protected bool ifEnableAngleCorrected = false; // 姿态角矫正开关
        //protected double detectingForceChangeTimesMax = 0.0; // 探测力变化最大倍数 
        //protected double detectingForceChangeSwitchAngle = 0.0; // 探测力变化开关角度
        //protected double detectingForceChangeDecayAngle = 0.0; // 探测力变化衰减角度

        protected double vibratingAttitudeMax = 0.0; // 探测姿态角最大值
        protected double movingSpeed = 0.0; // 移动速度
        protected double detectingBasicPreservedForce = 0.0; // 探测基准保持力大小
        //protected bool ifEnableDetectingForceChangeAtTransitionalPart = false; // 过渡段探测力变化开关
        //protected const double detectingForceChangeAtTransitionalPartDeclineProportion = 0.2; // 过渡段探测力变化开关
        protected double vibratingAttitudeMaxAtSmoothPart = 0.0; // 平滑段摆动姿态角最大值
        protected double vibratingAttitudeMinAtSteepPart = 0.0; // 陡峭段摆动姿态角最小值
        protected double vibratingAttitudeMaxAtSteepPart = 0.0; // 陡峭段摆动姿态角最大值
        #endregion

        #region 高级控制字段
        protected double nippleForbiddenRadius = 0.0; // 乳头防撞禁止半径
        protected double movingStopDistance = 0.0; // 移动方向停止距离
        protected double detectingStopDistance = 0.0; // 探测方向停止距离
        protected double detectingSafetyLiftDistance = 0.0; // 探测方向安全上升距离
        protected bool ifEnableInitialDetectingForce = false; // 初始力探测矫正初始姿态开关
        protected double detectingSinkDistance = 0.0; // 探测方向下沉距离

        protected VibratingMagnitude vibratingAngleDegree = VibratingMagnitude.Medium; // 摆动方向摆动幅度
        protected MovingLevel movingSpeedDegree = MovingLevel.Medium; // 移动方向移动快慢
        protected DetectingIntensity detectingForceDegree = DetectingIntensity.SlightlyLight; // 探测方向力度大小
        protected AligningDegree detectingAlignDegree = AligningDegree.Tight; // 探测整体贴合程度

        protected double movingUpEdgeDistance = 0.0; // 移动方向上边界距离
        protected double movingLeftEdgeDistance = 0.0; // 移动方向左边界距离
        protected double movingDownEdgeDistance = 0.0; // 移动方向下边界距离
        protected double movingRightEdgeDistance = 0.0; // 移动方向右边界距离

        protected double[] nippleTcpPostion = new double[6]; // 乳头对应的Tcp位置
        protected ScanningRegion ifCheckRightGalactophore = ScanningRegion.RightGalactophore; // 是否检测右侧乳房
        protected IdentifyBoundary identifyEdgeMode = IdentifyBoundary.OnlyUpBoundary; // 获得边界的方法

        protected double checkingStep = 0.0; // 扫查过程的步长
        #endregion

        #region 配置参数校验字段
        protected const double detectingErrorForceMinLowerBound = 0.0; // 探测方向误差力最小值下限
        protected const double detectingErrorForceMinUpperBound = 1.0; // 探测方向误差力最小值上限
        protected const double detectingErrorForceMaxLowerBound = 1.5; // 探测方向误差力最大值下限
        protected const double detectingErrorForceMaxUpperBound = 3.0; // 探测方向误差力最大值上限
        protected const double detectingSpeedMinLowerBound = 0.0000; // 探测方向运动速度最小值下限
        protected const double detectingSpeedMinUpperBound = 0.0003; // 探测方向运动速度最小值上限
        protected const double nippleForbiddenRadiusLowerBound = 0.010; // 乳头防撞禁止半径下限
        protected const double nippleForbiddenRadiusUpperBound = 0.030; // 乳头防撞禁止半径下限
        protected const double detectingSafetyLiftDistanceLowerBound = 0.010; // 探测方向安全上升距离下限
        protected const double detectingSafetyLiftDistanceUpperBound = 0.080; // 探测方向安全上升距离上限
        protected const double detectingStopDistanceLowerBound = 0.000; // 探测方向停止距离下限
        protected const double detectingStopDistanceUpperBound = 0.050; // 探测方向停止距离上限
        protected const double movingEdgeDistanceLowerBound = 0.000; // 移动方向边界距离下限
        protected const double movingUpEdgeDistanceUpperBound = 0.140; // 移动方向上边界距离上限
        protected const double movingLeftEdgeDistanceUpperBound = 0.080; // 移动方向左边界距离上限
        protected const double movingDownEdgeDistanceUpperBound = 0.060; // 移动方向下边界距离上限
        protected const double movingRightEdgeDistanceUpperBound = 0.060; // 移动方向右边界距离上限
        protected const double movingStopDistanceLowerBound = 0.000; // 移动方向停止距离下限
        protected const double movingStopDistanceUpperBound = 0.150; // 移动方向停止距离上限
        protected const double vibratingAttitudeMaxAtSmoothPartLowerBound = 0.0000; // 平滑段摆动姿态角最大值下限
        protected const double vibratingAttitudeMaxAtSmoothPartUpperBound = Math.PI / 6.0; // 平滑段摆动姿态角最大值上限
        protected const double vibratingAttitudeMinAtSteepPartLowerBound = Math.PI / 6.0; // 陡峭段摆动姿态角最小值下限
        protected const double vibratingAttitudeMinAtSteepPartUpperBound = Math.PI / 4.0; // 陡峭段摆动姿态角最小值上限
        protected const double vibratingAttitudeMaxAtSteepPartLowerBound = Math.PI / 4.0; // 陡峭段摆动姿态角最大值下限
        protected const double vibratingAttitudeMaxAtSteepPartUpperBound = Math.PI * 55.0 / 180.0; // 陡峭段摆动姿态角最大值上限
        protected const double checkingStepLowerBound = Math.PI / 12.0; // 扫查过程的步长下限
        protected const double checkingStepUpperBound = Math.PI / 3.0; // 扫查过程的步长上限
        #endregion

        #region 参数计算辅助字段
        protected const double vibratingAttitudeMaxAtSmoothPartUpperBoundForSmallVibratingMagnitude = 0.3491; // 平滑段摆动姿态角最大值在小摆动幅度下的上限
        protected const double vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude = 0.6981; // 陡峭段摆动姿态角最小值在中摆动幅度下的上限
        protected const double vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude = 0.8727; // 陡峭段摆动姿态角最小值在中摆动幅度下的上限
        protected const double detectingSinkDistanceForLightDetectingIntensity = 0.013; // 探测方向下沉距离在轻探测力度下的值
        protected const double detectingSinkDistanceForSlightlyLightDetectingIntensity = 0.014; // 探测方向下沉距离在稍轻探测力度下的值
        protected const double detectingSinkDistanceForSlightlyHeavyDetectingIntensity = 0.015; // 探测方向下沉距离在稍重探测力度下的值
        protected const double detectingSinkDistanceForHeavyDetectingIntensity = 0.016; // 探测方向下沉距离在重探测力度下的值
        protected const double movingUpEdgeDistanceTemplateValue = 0.120; // 移动方向上边界距离模板值
        protected const double movingUpLeftEdgeDistanceTemplateValue = 0.090; // 移动方向左上边界距离模板值
        protected const double movingLeftEdgeDistanceTemplateValue = 0.060; // 移动方向左边界距离模板值
        protected const double movingLeftDownEdgeDistanceTemplateValue = 0.030; // 移动方向左下边界距离模板值
        protected const double movingDownEdgeDistanceTemplateValue = 0.020; // 移动方向下边界距离模板值
        protected const double movingRightDownEdgeDistanceTemplateValue = 0.022; // 移动方向右下边界距离模板值
        protected const double movingRightEdgeDistanceTemplateValue = 0.040; // 移动方向右边界距离模板值
        protected const double movingUpRightEdgeDistanceTemplateValue = 0.080; // 移动方向右上边界距离模板值

        protected const double movingLengthForMaxMovingSpeed = 0.110; // 最大移动速度对应的移动距离
        protected const double movingLengthForMinMovingSpeed = 0.020; // 最小移动速度对应的移动距离
        protected const double movingSpeedUpperBound = 0.0008; // 移动速度上限
        protected const double movingSpeedLowerBound = 0.0002; // 移动速度下限
        protected const double movingSpeedGradingValue = 0.0001; // 移动速度最小分度值
        #endregion

        #region 属性
        /// <summary>
        /// 是否找到了乳头位置
        /// </summary>
        public bool IfNipplePositionFound
        {
            get { return ifNipplePositionFound; }
        }

        /// <summary>
        /// 是否完整扫描
        /// </summary>
        public bool IfEntireScan
        {
            get { return ifEntireScan; }
            set { ifEntireScan = value; }
        }

        /// <summary>
        /// 扫描角度
        /// </summary>
        public double AngleScan
        {
            get { return angleScan; }
            set { angleScan = value; }
        }

        /// <summary>
        /// 是否改变姿态
        /// </summary>
        public bool IfAttitudeChange
        {
            get { return ifAttitudeChange; }
            set { ifAttitudeChange = value; }
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
        public GalactophoreDetector(URDataProcessor InternalProcessor, double[] RecordedJointAngles, double[] InstallTcpPosition, bool InstallHanged, double ToolMass) :
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
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 3), new string[] { "False" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 4), new string[] { vibratingAttitudeMaxAtSmoothPartUpperBound.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { (nippleForbiddenRadiusLowerBound + 0.005).ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { movingUpEdgeDistanceTemplateValue.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { (detectingStopDistanceUpperBound - 0.011).ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { (detectingSafetyLiftDistanceLowerBound + 0.005).ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 11), new string[] { "False" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 12), new string[] { detectingSinkDistanceForSlightlyLightDetectingIntensity.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 13), new string[] { Enum.GetName(typeof(VibratingMagnitude), VibratingMagnitude.Medium) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 14), new string[] { Enum.GetName(typeof(MovingLevel), MovingLevel.Medium) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 15), new string[] { Enum.GetName(typeof(DetectingIntensity), DetectingIntensity.SlightlyLight) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 16), new string[] { Enum.GetName(typeof(AligningDegree), AligningDegree.Tight) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 17), new string[] { movingUpEdgeDistanceTemplateValue.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 18), new string[] { movingLeftEdgeDistanceTemplateValue.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 19), new string[] { movingDownEdgeDistanceTemplateValue.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 20), new string[] { movingRightEdgeDistanceTemplateValue.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 21), new string[] { "True" });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 22), new string[] { Enum.GetName(typeof(ScanningRegion), ScanningRegion.RightGalactophore) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 23), new string[] { Enum.GetName(typeof(IdentifyBoundary), IdentifyBoundary.AllBoundary) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 24), new string[] { checkingStepLowerBound.ToString("0.0000") });

            xmlProcessor = new XMLConnector(xmlFileName, replaceFilePath, parametersDictionary);
        }

        /// <summary>
        /// 从string列表中获得并保存到模块参数
        /// </summary>
        /// <param name="ParameterStringList">参数string列表</param>
        protected override void GetParametersFromString(List<string> ParameterStringList)
        {
            double tempValue = 0.0;
            detectingErrorForceMin = double.Parse(ParameterStringList[(int)ParameterList.DetectingErrorForceMin]);
            detectingErrorForceMax = double.Parse(ParameterStringList[(int)ParameterList.DetectingErrorForceMax]);
            detectingSpeedMin = double.Parse(ParameterStringList[(int)ParameterList.DetectingSpeedMin]);
            ifEnableAngleCorrected = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableAngleCorrected]);

            tempValue = double.Parse(ParameterStringList[(int)ParameterList.VibratingAttitudeMaxAtSmoothPart]);
            vibratingAttitudeMaxAtSmoothPart = tempValue <= notChange ? vibratingAttitudeMaxAtSmoothPart : tempValue;
            tempValue = double.Parse(ParameterStringList[(int)ParameterList.VibratingAttitudeMinAtSteepPart]);
            vibratingAttitudeMinAtSteepPart = tempValue <= notChange ? vibratingAttitudeMinAtSteepPart : tempValue;
            tempValue = double.Parse(ParameterStringList[(int)ParameterList.VibratingAttitudeMaxAtSteepPart]);
            vibratingAttitudeMaxAtSteepPart = tempValue <= notChange ? vibratingAttitudeMaxAtSteepPart : tempValue;

            nippleForbiddenRadius = double.Parse(ParameterStringList[(int)ParameterList.NippleForbiddenRadius]);

            tempValue = double.Parse(ParameterStringList[(int)ParameterList.MovingStopDistance]);
            movingStopDistance = tempValue <= notChange ? movingStopDistance : tempValue;

            detectingStopDistance = double.Parse(ParameterStringList[(int)ParameterList.DetectingStopDistance]);
            detectingSafetyLiftDistance = double.Parse(ParameterStringList[(int)ParameterList.DetectingSafetyLiftDistance]);
            ifEnableInitialDetectingForce = bool.Parse(ParameterStringList[(int)ParameterList.IfEnableInitialDetectingForce]);
            detectingSinkDistance = double.Parse(ParameterStringList[(int)ParameterList.DetectingSinkDistance]);
            vibratingAngleDegree = (VibratingMagnitude)byte.Parse(ParameterStringList[(int)ParameterList.VibratingAngleDegree]);
            movingSpeedDegree = (MovingLevel)byte.Parse(ParameterStringList[(int)ParameterList.MovingSpeedDegree]);
            detectingForceDegree = (DetectingIntensity)byte.Parse(ParameterStringList[(int)ParameterList.DetectingForceDegree]);
            detectingAlignDegree = (AligningDegree)byte.Parse(ParameterStringList[(int)ParameterList.DetectingAlignDegree]);

            tempValue = double.Parse(ParameterStringList[(int)ParameterList.MovingUpEdgeDistance]);
            movingUpEdgeDistance = tempValue <= notChange ? movingUpEdgeDistance : tempValue;
            tempValue = double.Parse(ParameterStringList[(int)ParameterList.MovingLeftEdgeDistance]);
            movingLeftEdgeDistance = tempValue <= notChange ? movingLeftEdgeDistance : tempValue;
            tempValue = double.Parse(ParameterStringList[(int)ParameterList.MovingDownEdgeDistance]);
            movingDownEdgeDistance = tempValue <= notChange ? movingDownEdgeDistance : tempValue;
            tempValue = double.Parse(ParameterStringList[(int)ParameterList.MovingRightEdgeDistance]);
            movingRightEdgeDistance = tempValue <= notChange ? movingRightEdgeDistance : tempValue;

            ifAutoReplaceConfiguration = bool.Parse(ParameterStringList[(int)ParameterList.IfAutoReplaceConfiguration]);
            ifCheckRightGalactophore = (ScanningRegion)int.Parse(ParameterStringList[(int)ParameterList.IfCheckRightGalactophore]);
            identifyEdgeMode = (IdentifyBoundary)byte.Parse(ParameterStringList[(int)ParameterList.IdentifyBoundaryMode]);
            checkingStep = double.Parse(ParameterStringList[(int)ParameterList.CheckingStep]);

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
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 3), new string[] { ifEnableAngleCorrected.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 4), new string[] { vibratingAttitudeMaxAtSmoothPart.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 5), new string[] { vibratingAttitudeMinAtSteepPart.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 6), new string[] { vibratingAttitudeMaxAtSteepPart.ToString("0.0000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 7), new string[] { nippleForbiddenRadius.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 8), new string[] { movingStopDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 9), new string[] { detectingStopDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 10), new string[] { detectingSafetyLiftDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 11), new string[] { ifEnableInitialDetectingForce.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 12), new string[] { detectingSinkDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 13), new string[] { Enum.GetName(typeof(VibratingMagnitude), vibratingAngleDegree) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 14), new string[] { Enum.GetName(typeof(MovingLevel), movingSpeedDegree) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 15), new string[] { Enum.GetName(typeof(DetectingIntensity), detectingForceDegree) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 16), new string[] { Enum.GetName(typeof(AligningDegree), detectingAlignDegree) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 17), new string[] { movingUpEdgeDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 18), new string[] { movingLeftEdgeDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 19), new string[] { movingDownEdgeDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 20), new string[] { movingRightEdgeDistance.ToString("0.000") });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 21), new string[] { ifAutoReplaceConfiguration.ToString() });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 22), new string[] { Enum.GetName(typeof(ScanningRegion), ifCheckRightGalactophore) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 23), new string[] { Enum.GetName(typeof(IdentifyBoundary), identifyEdgeMode) });
            parametersDictionary.Add(Enum.GetName(typeof(ParameterList), 24), new string[] { checkingStep.ToString("0.0000") });

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
            ifEnableAngleCorrected = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableAngleCorrected)][0]);
            vibratingAttitudeMaxAtSmoothPart = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.VibratingAttitudeMaxAtSmoothPart)][0]);
            vibratingAttitudeMinAtSteepPart = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.VibratingAttitudeMinAtSteepPart)][0]);
            vibratingAttitudeMaxAtSteepPart = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.VibratingAttitudeMaxAtSteepPart)][0]);
            nippleForbiddenRadius = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.NippleForbiddenRadius)][0]);
            movingStopDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingStopDistance)][0]);
            detectingStopDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingStopDistance)][0]);
            detectingSafetyLiftDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingSafetyLiftDistance)][0]);
            ifEnableInitialDetectingForce = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfEnableInitialDetectingForce)][0]);
            detectingSinkDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingSinkDistance)][0]);
            vibratingAngleDegree = (VibratingMagnitude)Enum.Parse(typeof(VibratingMagnitude), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.VibratingAngleDegree)][0]);
            movingSpeedDegree = (MovingLevel)Enum.Parse(typeof(MovingLevel), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingSpeedDegree)][0]);
            detectingForceDegree = (DetectingIntensity)Enum.Parse(typeof(DetectingIntensity), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingForceDegree)][0]);
            detectingAlignDegree = (AligningDegree)Enum.Parse(typeof(AligningDegree), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.DetectingAlignDegree)][0]);
            movingUpEdgeDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingUpEdgeDistance)][0]);
            movingLeftEdgeDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingLeftEdgeDistance)][0]);
            movingDownEdgeDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingDownEdgeDistance)][0]);
            movingRightEdgeDistance = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.MovingRightEdgeDistance)][0]);
            ifAutoReplaceConfiguration = bool.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfAutoReplaceConfiguration)][0]);
            ifCheckRightGalactophore = (ScanningRegion)Enum.Parse(typeof(ScanningRegion), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IfCheckRightGalactophore)][0]);
            identifyEdgeMode = (IdentifyBoundary)Enum.Parse(typeof(IdentifyBoundary), parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.IdentifyBoundaryMode)][0]);
            checkingStep = double.Parse(parametersDictionary[Enum.GetName(typeof(ParameterList), ParameterList.CheckingStep)][0]);

            CalculateAndCheckParametersBothExposedAndHidden(); // 加载后限制部分参数并计算相关参数
        }

        /// <summary>
        /// 计算并检查所有相关配置参数
        /// </summary>
        protected override void CalculateAndCheckParametersBothExposedAndHidden()
        {
            CheckParametersWithLimitations(); // 限制
            CalculateParametersFromCorrespondedParameters(); // 计算
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
            parametersList.Add(new string[] { ifEnableAngleCorrected.ToString() });
            parametersList.Add(new string[] { vibratingAttitudeMaxAtSmoothPart.ToString("0.0000") });
            parametersList.Add(new string[] { vibratingAttitudeMinAtSteepPart.ToString("0.0000") });
            parametersList.Add(new string[] { vibratingAttitudeMaxAtSteepPart.ToString("0.0000") });
            parametersList.Add(new string[] { nippleForbiddenRadius.ToString("0.000") });
            parametersList.Add(new string[] { movingStopDistance.ToString("0.000") });
            parametersList.Add(new string[] { detectingStopDistance.ToString("0.000") });
            parametersList.Add(new string[] { detectingSafetyLiftDistance.ToString("0.000") });
            parametersList.Add(new string[] { ifEnableInitialDetectingForce.ToString() });
            parametersList.Add(new string[] { detectingSinkDistance.ToString("0.000") });
            parametersList.Add(new string[] { Enum.GetName(typeof(VibratingMagnitude), vibratingAngleDegree) });
            parametersList.Add(new string[] { Enum.GetName(typeof(MovingLevel), movingSpeedDegree) });
            parametersList.Add(new string[] { Enum.GetName(typeof(DetectingIntensity), detectingForceDegree) });
            parametersList.Add(new string[] { Enum.GetName(typeof(AligningDegree), detectingAlignDegree) });
            parametersList.Add(new string[] { movingUpEdgeDistance.ToString("0.000") });
            parametersList.Add(new string[] { movingLeftEdgeDistance.ToString("0.000") });
            parametersList.Add(new string[] { movingDownEdgeDistance.ToString("0.000") });
            parametersList.Add(new string[] { movingRightEdgeDistance.ToString("0.000") });
            parametersList.Add(new string[] { ifAutoReplaceConfiguration.ToString() });
            parametersList.Add(new string[] { Enum.GetName(typeof(ScanningRegion), ifCheckRightGalactophore) });
            parametersList.Add(new string[] { Enum.GetName(typeof(IdentifyBoundary), identifyEdgeMode) });
            parametersList.Add(new string[] { checkingStep.ToString("0.0000") });

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
            if (nippleForbiddenRadius < nippleForbiddenRadiusLowerBound) nippleForbiddenRadius = nippleForbiddenRadiusLowerBound;
            if (nippleForbiddenRadius > nippleForbiddenRadiusUpperBound) nippleForbiddenRadius = nippleForbiddenRadiusUpperBound;
            if (detectingSafetyLiftDistance < detectingSafetyLiftDistanceLowerBound) detectingSafetyLiftDistance = detectingSafetyLiftDistanceLowerBound;
            if (detectingSafetyLiftDistance > detectingSafetyLiftDistanceUpperBound) detectingSafetyLiftDistance = detectingSafetyLiftDistanceUpperBound;
            if (detectingStopDistance < detectingStopDistanceLowerBound) detectingStopDistance = detectingStopDistanceLowerBound;
            if (detectingStopDistance > detectingStopDistanceUpperBound) detectingStopDistance = detectingStopDistanceUpperBound;
            if (movingUpEdgeDistance < movingEdgeDistanceLowerBound) movingUpEdgeDistance = movingEdgeDistanceLowerBound;
            if (movingUpEdgeDistance > movingUpEdgeDistanceUpperBound) movingUpEdgeDistance = movingUpEdgeDistanceUpperBound;
            if (movingLeftEdgeDistance < movingEdgeDistanceLowerBound) movingLeftEdgeDistance = movingEdgeDistanceLowerBound;
            if (movingLeftEdgeDistance > movingLeftEdgeDistanceUpperBound) movingLeftEdgeDistance = movingLeftEdgeDistanceUpperBound;
            if (movingDownEdgeDistance < movingEdgeDistanceLowerBound) movingDownEdgeDistance = movingEdgeDistanceLowerBound;
            if (movingDownEdgeDistance > movingDownEdgeDistanceUpperBound) movingDownEdgeDistance = movingDownEdgeDistanceUpperBound;
            if (movingRightEdgeDistance < movingEdgeDistanceLowerBound) movingRightEdgeDistance = movingEdgeDistanceLowerBound;
            if (movingRightEdgeDistance > movingRightEdgeDistanceUpperBound) movingRightEdgeDistance = movingRightEdgeDistanceUpperBound;
            if (movingStopDistance < movingStopDistanceLowerBound) movingStopDistance = movingStopDistanceLowerBound;
            if (movingStopDistance > movingStopDistanceUpperBound) movingStopDistance = movingStopDistanceUpperBound;
            if (vibratingAttitudeMaxAtSmoothPart < vibratingAttitudeMaxAtSmoothPartLowerBound) vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPartLowerBound;
            if (vibratingAttitudeMaxAtSmoothPart > vibratingAttitudeMaxAtSmoothPartUpperBound) vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPartUpperBound;
            if (vibratingAttitudeMinAtSteepPart < vibratingAttitudeMinAtSteepPartLowerBound) vibratingAttitudeMinAtSteepPart = vibratingAttitudeMinAtSteepPartLowerBound;
            if (vibratingAttitudeMinAtSteepPart > vibratingAttitudeMinAtSteepPartUpperBound) vibratingAttitudeMinAtSteepPart = vibratingAttitudeMinAtSteepPartUpperBound;
            if (vibratingAttitudeMaxAtSteepPart < vibratingAttitudeMaxAtSteepPartLowerBound) vibratingAttitudeMaxAtSteepPart = vibratingAttitudeMaxAtSteepPartLowerBound;
            if (vibratingAttitudeMaxAtSteepPart > vibratingAttitudeMaxAtSteepPartUpperBound) vibratingAttitudeMaxAtSteepPart = vibratingAttitudeMaxAtSteepPartUpperBound;

            if (detectingAlignDegree == AligningDegree.Loose && detectingForceDegree == DetectingIntensity.Heavy)
            {
                detectingForceDegree = DetectingIntensity.SlightltHeavy;
            }
            if (detectingAlignDegree == AligningDegree.Tight && detectingForceDegree == DetectingIntensity.Light)
            {
                detectingForceDegree = DetectingIntensity.SlightlyLight;
            }

            if (checkingStep < checkingStepLowerBound) checkingStep = checkingStepLowerBound;
            if (checkingStep > checkingStepUpperBound) checkingStep = checkingStepUpperBound;
        }

        /// <summary>
        /// 通过部分参数计算相应参数
        /// </summary>
        protected void CalculateParametersFromCorrespondedParameters()
        {
            CalculateMovingEdgeDistance();
            CalculateDetectingBasicPreservedForce();
            CalculateVibratingAttitudeParameters();
            CalculateMovingStopDistance();
            CalculateDetectingSinkDistance();
            CalculateInterpolatedReferenceCurve();
        }

        /// <summary>
        /// 计算移动边界距离
        /// </summary>
        protected virtual void CalculateMovingEdgeDistance()
        {
            switch (identifyEdgeMode)
            {
                case IdentifyBoundary.OnlyUpBoundary:
                    movingDownEdgeDistance = movingUpEdgeDistance / movingUpEdgeDistanceTemplateValue * movingDownEdgeDistanceTemplateValue;
                    movingLeftEdgeDistance = movingUpEdgeDistance / movingUpEdgeDistanceTemplateValue * movingLeftEdgeDistanceTemplateValue;
                    movingRightEdgeDistance = movingUpEdgeDistance / movingUpEdgeDistanceTemplateValue * movingRightEdgeDistanceTemplateValue;
                    break;
                case IdentifyBoundary.UpDownBoundary:
                    movingLeftEdgeDistance = (movingUpEdgeDistance / movingUpEdgeDistanceTemplateValue + movingDownEdgeDistance / movingDownEdgeDistanceTemplateValue) * movingLeftEdgeDistanceTemplateValue / 2.0;
                    movingRightEdgeDistance = (movingUpEdgeDistance / movingUpEdgeDistanceTemplateValue + movingDownEdgeDistance / movingDownEdgeDistanceTemplateValue) * movingRightEdgeDistanceTemplateValue / 2.0;
                    break;
                case IdentifyBoundary.AllBoundary:
                    break;
                default:
                    movingUpEdgeDistance = movingUpEdgeDistanceTemplateValue;
                    movingDownEdgeDistance = movingDownEdgeDistanceTemplateValue;
                    movingLeftEdgeDistance = movingLeftEdgeDistanceTemplateValue;
                    movingRightEdgeDistance = movingRightEdgeDistanceTemplateValue;
                    break;
            }
        }

        /// <summary>
        /// 计算探测基准保持力大小
        /// </summary>
        protected virtual void CalculateDetectingBasicPreservedForce()
        {
            switch (detectingForceDegree)
            {
                case DetectingIntensity.Light:
                    detectingBasicPreservedForce = 3.0;
                    break;
                case DetectingIntensity.SlightlyLight:
                    detectingBasicPreservedForce = 4.0;
                    break;
                case DetectingIntensity.SlightltHeavy:
                    detectingBasicPreservedForce = 5.0;
                    break;
                case DetectingIntensity.Heavy:
                    detectingBasicPreservedForce = 6.0;
                    break;
                default:
                    detectingBasicPreservedForce = 3.0;
                    break;
            }
        }

        /// <summary>
        /// 计算摆动姿态角极限值
        /// </summary>
        protected virtual void CalculateVibratingAttitudeParameters()
        {
            switch (vibratingAngleDegree)
            {
                case VibratingMagnitude.Small:
                    vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPart > vibratingAttitudeMaxAtSmoothPartUpperBoundForSmallVibratingMagnitude ? vibratingAttitudeMaxAtSmoothPartUpperBoundForSmallVibratingMagnitude : vibratingAttitudeMaxAtSmoothPart;
                    vibratingAttitudeMinAtSteepPart = vibratingAttitudeMinAtSteepPartLowerBound;
                    vibratingAttitudeMaxAtSteepPart = vibratingAttitudeMaxAtSteepPartLowerBound;
                    break;
                case VibratingMagnitude.Medium:
                    vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPart < vibratingAttitudeMaxAtSmoothPartUpperBoundForSmallVibratingMagnitude ? vibratingAttitudeMaxAtSmoothPartUpperBound : vibratingAttitudeMaxAtSmoothPart;
                    vibratingAttitudeMinAtSteepPart = (vibratingAttitudeMinAtSteepPart > vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude || vibratingAttitudeMinAtSteepPart <= vibratingAttitudeMinAtSteepPartLowerBound) ? vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude : vibratingAttitudeMinAtSteepPart;
                    vibratingAttitudeMaxAtSteepPart = (vibratingAttitudeMaxAtSteepPart > vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude || vibratingAttitudeMaxAtSteepPart <= vibratingAttitudeMaxAtSteepPartLowerBound) ? vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude : vibratingAttitudeMaxAtSteepPart;
                    break;
                case VibratingMagnitude.Large:
                    vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPartUpperBound;
                    vibratingAttitudeMinAtSteepPart = vibratingAttitudeMinAtSteepPart < vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude ? vibratingAttitudeMinAtSteepPartUpperBound : vibratingAttitudeMinAtSteepPart;
                    vibratingAttitudeMaxAtSteepPart = vibratingAttitudeMaxAtSteepPart < vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude ? vibratingAttitudeMaxAtSteepPartUpperBound : vibratingAttitudeMaxAtSteepPart;
                    break;
                default:
                    vibratingAttitudeMaxAtSmoothPart = vibratingAttitudeMaxAtSmoothPartUpperBound;
                    vibratingAttitudeMinAtSteepPart = vibratingAttitudeMinAtSteepPartUpperBoundForMediumVibratingMagnitude;
                    vibratingAttitudeMaxAtSteepPart = vibratingAttitudeMaxAtSteepPartUpperBoundForMediumVibratingMagnitude;
                    break;
            }
        }

        /// <summary>
        /// 计算移动方向停止距离
        /// </summary>
        protected virtual void CalculateMovingStopDistance()
        {
            movingStopDistance = movingUpEdgeDistance;
            movingStopDistance = Math.Max(movingStopDistance, movingLeftEdgeDistance);
            movingStopDistance = Math.Max(movingStopDistance, movingDownEdgeDistance);
            movingStopDistance = Math.Max(movingStopDistance, movingRightEdgeDistance);
        }

        /// <summary>
        /// 计算探测方向下沉距离
        /// </summary>
        protected virtual void CalculateDetectingSinkDistance()
        {
            switch (detectingForceDegree)
            {
                case DetectingIntensity.Light:
                    detectingSinkDistance = detectingSinkDistanceForLightDetectingIntensity;
                    break;
                case DetectingIntensity.SlightlyLight:
                    detectingSinkDistance = detectingSinkDistanceForSlightlyLightDetectingIntensity;
                    break;
                case DetectingIntensity.SlightltHeavy:
                    detectingSinkDistance = detectingSinkDistanceForSlightlyHeavyDetectingIntensity;
                    break;
                case DetectingIntensity.Heavy:
                    detectingSinkDistance = detectingSinkDistanceForHeavyDetectingIntensity;
                    break;
                default:
                    detectingSinkDistance = detectingSinkDistanceForSlightlyLightDetectingIntensity;
                    break;
            }
        }

        /// <summary>
        /// 计算插值参考边界曲线
        /// </summary>
        protected virtual void CalculateInterpolatedReferenceCurve()
        {
            // 构造待插值的数列
            double[] interpolatedAngle = new double[9];
            double[] interpolatedLength = new double[9];

            for (int i = 0; i < 9; i++)
            {
                interpolatedAngle[i] = Math.PI / 4.0 * i;
            }
            interpolatedLength[0] = movingRightEdgeDistance;
            interpolatedLength[2] = movingUpEdgeDistance;
            interpolatedLength[4] = movingLeftEdgeDistance;
            interpolatedLength[6] = movingDownEdgeDistance;
            interpolatedLength[8] = movingRightEdgeDistance;
            interpolatedLength[1] = movingUpRightEdgeDistanceTemplateValue *
                                                   (interpolatedLength[0] / movingRightEdgeDistanceTemplateValue * (Math.PI / 2.0 - interpolatedAngle[1]) / (Math.PI / 2.0) +
                                                   interpolatedLength[2] / movingUpEdgeDistanceTemplateValue * (interpolatedAngle[1] - 0.0) / (Math.PI / 2.0));
            interpolatedLength[3] = movingUpLeftEdgeDistanceTemplateValue *
                                                   (interpolatedLength[2] / movingUpEdgeDistanceTemplateValue * (Math.PI - interpolatedAngle[3]) / (Math.PI / 2.0) +
                                                   interpolatedLength[4] / movingLeftEdgeDistanceTemplateValue * (interpolatedAngle[3] - Math.PI / 2.0) / (Math.PI / 2.0));
            interpolatedLength[5] = movingLeftDownEdgeDistanceTemplateValue *
                                                   (interpolatedLength[4] / movingLeftEdgeDistanceTemplateValue * (Math.PI * 1.5 - interpolatedAngle[5]) / (Math.PI / 2.0) +
                                                   interpolatedLength[6] / movingDownEdgeDistanceTemplateValue * (interpolatedAngle[5] - Math.PI) / (Math.PI / 2.0));
            interpolatedLength[7] = movingRightDownEdgeDistanceTemplateValue *
                                                   (interpolatedLength[6] / movingDownEdgeDistanceTemplateValue * (Math.PI * 2.0 - interpolatedAngle[7]) / (Math.PI / 2.0) +
                                                   interpolatedLength[8] / movingRightEdgeDistanceTemplateValue * (interpolatedAngle[7] - Math.PI * 1.5) / (Math.PI / 2.0));
            for (int k = 1; k < 9; k += 2)
            {
                interpolatedLength[k] = interpolatedLength[k] > Math.Max(interpolatedLength[k - 1], interpolatedLength[k + 1]) ? Math.Max(interpolatedLength[k - 1], interpolatedLength[k + 1]) : interpolatedLength[k];
                interpolatedLength[k] = interpolatedLength[k] < Math.Min(interpolatedLength[k - 1], interpolatedLength[k + 1]) ? Math.Min(interpolatedLength[k - 1], interpolatedLength[k + 1]) : interpolatedLength[k];
            }

            double[,] coefficientMatrix = URMath.SimpleCubicSplineCirculatedInterpolation(interpolatedAngle, interpolatedLength);
            for (int j = 0; j < 8; j++)
            {
                cubicSplineCurveConstantTerm[j] = coefficientMatrix[0, j];
                cubicSplineCurveLinearTermCoefficient[j] = coefficientMatrix[1, j];
                cubicSplineCurveQuadraticTermCoefficient[j] = coefficientMatrix[2, j];
                cubicSplineCurveCubicTermCoefficient[j] = coefficientMatrix[3, j];
            }
        }

        /// <summary>
        /// 获得所给角度对应的三次插值参考长度
        /// </summary>
        /// <param name="Angle">所给的角度</param>
        /// <returns>对应的参考长度</returns>
        protected virtual double GetReferenceLengthFromInterpolatedCurve(double Angle)
        {
            if (Angle < 0.0) Angle = 0.0;
            if (Angle > Math.PI * 2.0) Angle = Math.PI * 2.0;
            int usingCoefficientNumber = (int)Math.Floor(Angle / (Math.PI / 4.0));
            usingCoefficientNumber = usingCoefficientNumber > 7 ? 7 : usingCoefficientNumber;

            double[] angleUsed = { 0.0, Math.PI * 0.25, Math.PI * 0.5, Math.PI * 0.75, Math.PI, Math.PI * 1.25, Math.PI * 1.5, Math.PI * 1.75 };
            return cubicSplineCurveConstantTerm[usingCoefficientNumber] +
                       cubicSplineCurveLinearTermCoefficient[usingCoefficientNumber] * (Angle - angleUsed[usingCoefficientNumber]) +
                       cubicSplineCurveQuadraticTermCoefficient[usingCoefficientNumber] * Math.Pow(Angle - angleUsed[usingCoefficientNumber], 2.0) +
                       cubicSplineCurveCubicTermCoefficient[usingCoefficientNumber] * Math.Pow(Angle - angleUsed[usingCoefficientNumber], 3.0);
        }

        /// <summary>
        /// 计算单程路径所需的参数
        /// </summary>
        /// <param name="ThisRouteAngle">该程路径所占的角度</param>
        protected void CalculateSingleRouteParameters(double ThisRouteAngle)
        {
            // 限幅
            ThisRouteAngle = ThisRouteAngle > Math.PI * 2.0 ? Math.PI * 2.0 : ThisRouteAngle;
            ThisRouteAngle = ThisRouteAngle < 0.0 ? 0.0 : ThisRouteAngle;

            CalculateMovingSpeed(ThisRouteAngle);
            CalculateDetectingSpeedMax();
            CalculateVibratingSpeedMax(ThisRouteAngle);
            CalculateVibratingAttitudeMax(ThisRouteAngle);
            //CalculateVibratingAttitudeJudgeParameters();
            //CalculateDetectingForceChangeParameters(ThisRouteAngle);
        }

        /// <summary>
        /// 计算单程路径的移动速度
        /// </summary>
        /// <param name="ThisRouteAngle">该程路径所占的角度</param>
        protected virtual void CalculateMovingSpeed(double ThisRouteAngle)
        {
            // 获得该段参考长度
            double thisRouteLength = GetReferenceLengthFromInterpolatedCurve(ThisRouteAngle);

            // 获得参考长度对应的移动速度
            if (thisRouteLength > movingLengthForMaxMovingSpeed)
            {
                movingSpeed = movingSpeedUpperBound;
            }
            else if (thisRouteLength < movingLengthForMinMovingSpeed)
            {
                movingSpeed = movingSpeedLowerBound;
            }
            else
            {
                movingSpeed = (thisRouteLength - movingDownEdgeDistanceTemplateValue) / 150.0 + movingSpeedLowerBound;
            }

            // 修正移动速度
            switch (movingSpeedDegree)
            {
                case MovingLevel.Slow:
                    movingSpeed -= movingSpeedGradingValue;
                    movingSpeed = movingSpeed < movingSpeedLowerBound ? movingSpeedLowerBound : movingSpeed;
                    break;
                case MovingLevel.Fast:
                    movingSpeed += movingSpeedGradingValue;
                    movingSpeed = movingSpeed > movingSpeedUpperBound ? movingSpeedUpperBound : movingSpeed;
                    break;
                default:
                    break;
            }
            movingSpeed = Math.Round(movingSpeed * 10000.0) / 10000.0;
        }

        /// <summary>
        /// 计算单程路径的探测方向运动速度最大值
        /// </summary>
        protected virtual void CalculateDetectingSpeedMax()
        {
            detectingSpeedMax = movingSpeed * 31.0 / 6.0 - Math.Pow(movingSpeed, 2.0) * 10000.0 / 3.0;
            detectingSpeedMax = Math.Round(detectingSpeedMax * 10000.0) / 10000.0;
        }

        /// <summary>
        /// 计算单程路径的摆动方向运动速度最大值
        /// </summary>
        /// <param name="ThisRouteAngle">该程路径所占的角度</param>
        protected virtual void CalculateVibratingSpeedMax(double ThisRouteAngle)
        {
            vibratingSpeedMax = movingSpeed * 100.0 / 3.0 + 1.0 / 120.0;
            vibratingSpeedMax = Math.Round(vibratingSpeedMax * 10000.0) / 10000.0;
        }

        /// <summary>
        /// 计算单程路径的探测姿态角最大值
        /// </summary>
        /// <param name="ThisRouteAngle">该程路径所占的角度</param>
        protected virtual void CalculateVibratingAttitudeMax(double ThisRouteAngle)
        {
            if (ThisRouteAngle > Math.PI / 2.0 - smoothPartHalfAngle && ThisRouteAngle <= Math.PI + smoothPartHalfAngle)
            { // 平滑段
                vibratingAttitudeMax = vibratingAttitudeMaxAtSmoothPart;
            }
            else if (ThisRouteAngle >= Math.PI * 1.5 - steepPartHalfAngle && ThisRouteAngle <= Math.PI * 1.5 + steepPartHalfAngle)
            { // 陡峭段
                double angleDifference = Math.Abs(ThisRouteAngle - Math.PI * 1.5);
                vibratingAttitudeMax = vibratingAttitudeMaxAtSteepPart - angleDifference / steepPartHalfAngle * (vibratingAttitudeMaxAtSteepPart - vibratingAttitudeMinAtSteepPart);
            }
            else
            { // 过渡段
                double angleDifference = (Math.Abs(ThisRouteAngle - Math.PI * 1.5) > Math.PI ? (Math.PI * 2.0 - Math.Abs(ThisRouteAngle - Math.PI * 1.5)) : Math.Abs(ThisRouteAngle - Math.PI * 1.5)) - steepPartHalfAngle;
                vibratingAttitudeMax = vibratingAttitudeMinAtSteepPart - angleDifference / (Math.PI - smoothPartHalfAngle - steepPartHalfAngle) * (vibratingAttitudeMinAtSteepPart - vibratingAttitudeMaxAtSmoothPart);
            }
        }

        /// <summary>
        /// 计算单程路径的姿态判别用参数
        /// </summary>
        //protected virtual void CalculateVibratingAttitudeJudgeParameters()
        //{
        //    vibratingAttitudeJudgeSamplingNumber = 150 - (int)Math.Round(movingSpeed * 250000 / 2.0);
        //    vibratingAttitudeJudgeDifferenceInterval = (int)Math.Round((double)vibratingAttitudeJudgeSamplingNumber * 3.0 / 5.0);
        //    vibratingAttitudeJudgeExtensionPeriod = (int)Math.Round((double)vibratingAttitudeJudgeSamplingNumber * 4.0 / 5.0);
        //}

        /// <summary>
        /// 计算单程路径的探测力变化用参数，以及是否会用到
        /// </summary>
        /// <param name="ThisRouteAngle">该程路径所占的角度</param>
        //protected virtual void CalculateDetectingForceChangeParameters(double ThisRouteAngle)
        //{
        //    if (ThisRouteAngle > Math.PI / 2.0 - smoothPartHalfAngle && ThisRouteAngle <= Math.PI + smoothPartHalfAngle)
        //    { // 平滑段
        //        ifEnableDetectingForceChange = false;
        //    }
        //    else if (ThisRouteAngle >= Math.PI * 1.5 - steepPartHalfAngle && ThisRouteAngle <= Math.PI * 1.5 + steepPartHalfAngle)
        //    { // 陡峭段
        //        ifEnableDetectingForceChange = true;
        //        detectingForceChangeTimesMax = 3.48 - 0.7933 * detectingBasicPreservedForce + 0.12 * Math.Pow(detectingBasicPreservedForce, 2.0) - 0.006667 * Math.Pow(detectingBasicPreservedForce, 3.0);
        //    }
        //    else
        //    { // 过渡段
        //        if (ifEnableDetectingForceChangeAtTransitionalPart)
        //        {
        //            ifEnableDetectingForceChange = true;
        //            detectingForceChangeTimesMax = 3.48 - 0.7933 * detectingBasicPreservedForce + 0.12 * Math.Pow(detectingBasicPreservedForce, 2.0) - 0.006667 * Math.Pow(detectingBasicPreservedForce, 3.0);
        //            detectingForceChangeTimesMax = ((detectingForceChangeTimesMax - 1.0) * detectingForceChangeAtTransitionalPartDeclineProportion + 1.0);
        //        }
        //        else
        //        {
        //            ifEnableDetectingForceChange = false;
        //        }
        //    }
        //    detectingForceChangeDecayAngle = 0.173 * vibratingAttitudeMax + 0.083893;
        //    detectingForceChangeSwitchAngle = vibratingAttitudeMax - detectingForceChangeDecayAngle;
        //}

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
            ifNipplePositionFound = false;
        }

        /// <summary>
        /// 寻找乳头位置
        /// </summary>
        public void FindNippleTcpPosition()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionSetAndBegin(ServoFreeTranslation.ServoDirectionAtTcp.DirectionZ,
                                                                                                                                       findConfigurationParameterSpeedMax, findConfigurationParameterSpeedMin,
                                                                                                                                       findConfigurationParameterForceMax, findConfigurationParameterForceMin,
                                                                                                                                       findConfigurationParameterDistanceMax,
                                                                                                                                       false,
                                                                                                                                       ServoFreeTranslation.ServoDirectionAtTcp.DirectionX,
                                                                                                                                       0.0);

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin to find nipple position.");
                }));
            }
        }

        /// <summary>
        /// 确认找到了乳头位置
        /// </summary>
        public void ConfirmNippleTcpPositionFound()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionAbort();

                    Thread.Sleep(200);
                    nippleTcpPostion = internalProcessor.PositionsTcpActual;
                    ifNipplePositionFound = true;

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Nipple position is found and recorded.");
                }));
            }
        }

        /// <summary>
        ///  寻找安全抬升距离
        /// </summary>
        public void FindSafetyLiftDistance()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionSetAndBegin(ServoFreeTranslation.ServoDirectionAtTcp.DirectionZ,
                                                                                                                                       findConfigurationParameterSpeedMax, findConfigurationParameterSpeedMin,
                                                                                                                                       findConfigurationParameterForceMax, findConfigurationParameterForceMin,
                                                                                                                                       findConfigurationParameterDistanceMax,
                                                                                                                                       false,
                                                                                                                                       ServoFreeTranslation.ServoDirectionAtTcp.DirectionX,
                                                                                                                                       0.0);

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin to find safe lift distance.");
                }));
            }
        }

        /// <summary>
        /// 结束安全抬升距离的寻找
        /// </summary>
        public void EndFindSafetyLiftDistance()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionAbort();
                    Thread.Sleep(100);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Safe lift distance is recorded.");
                }));
            }
        }

        /// <summary>
        ///  寻找其余配置参数
        /// </summary>
        public void FindMostConfigurationParameters()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionSetAndBegin(ServoFreeTranslation.ServoDirectionAtTcp.DirectionY | ServoFreeTranslation.ServoDirectionAtTcp.DirectionZ,
                                                                                                                                       findConfigurationParameterSpeedMax, findConfigurationParameterSpeedMin,
                                                                                                                                       findConfigurationParameterForceMax, findConfigurationParameterForceMin,
                                                                                                                                       findConfigurationParameterDistanceMax,
                                                                                                                                       false,
                                                                                                                                       ServoFreeTranslation.ServoDirectionAtTcp.DirectionX,
                                                                                                                                       0.0);

                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Begin to find other configuration parameters.");
                }));
            }
        }

        /// <summary>
        /// 结束其余配置参数的寻找
        /// </summary>
        public void EndMostConfigurationParameters()
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                Task.Run(new Action(() =>
                {
                    internalProcessor.servoFreeTranslationModule.ServoMotionAbort();
                    Thread.Sleep(100);
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Other configuration parameters searching is finished.");
                }));
            }
        }

        /// <summary>
        /// 由纵切检查向横切检查转变
        /// </summary>
        /// <param name="Transverse">反转变化</param>
        /// <returns>返回运行的任务</returns>
        public Task LongitudinalToHorizontalCheck(bool Transverse = false)
        {
            if (workingStatus == WorkStatus.ParametersConfiguration)
            {
                if (Transverse)
                {
                    return Task.Run(new Action(() =>
                    {
                        double[] longitudinalPosition = internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistanceLowerBound, nippleTcpPostion);
                        internalProcessor.SendURCommanderMoveL(longitudinalPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Longitudinal checking position is arrived.");
                    }));
                }
                else
                {
                    return Task.Run(new Action(() =>
                    {
                        double[] horizontalPosition = internalProcessor.MoveAlongTcpZAxis(-detectingSafetyLiftDistanceLowerBound,
                                                                       internalProcessor.RotateByTcpZAxis(Math.PI / 2.0, nippleTcpPostion));
                        internalProcessor.SendURCommanderMoveL(horizontalPosition, normalMoveAccelerationL, normalMoveSpeedL);
                        Thread.Sleep(800);
                        if (!JudgeIfMotionCanBeContinued()) return;
                        while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                        {
                            Thread.Sleep(200);
                            if (!JudgeIfMotionCanBeContinued()) return;
                        }
                        if (!JudgeIfMotionCanBeContinued()) return;
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Horizontal checking position is arrived.");
                    }));
                }
            }
            else return null;
        }

        /// <summary>
        /// 扫描力度校验
        /// </summary>
        /// <param name="WithdrawRadius">避让半径，应该与测得的乳头防撞禁止半径相当</param>
        /// <param name="ForceDegreeNeed">需要停止探测时的力程度大小</param>
        /// <returns>返回运行的任务</returns>
        //public Task ScanForceCheck(double WithdrawRadius, DetectingIntensity ForceDegreeNeed)
        //{
        //    if (ifEnableDetectingForceCheck && workingStatus == WorkStatus.ParametersConfiguration)
        //    {
        //        return Task.Run(new Action(() =>
        //        {
        //            // 1. 移动到第一条扫查线开始位置的未下沉位置
        //            double[] firstIterationBeginPositionBeforeSink = internalProcessor.MoveAlongTcpYAxis(((int)ScanningProcess.FrontHalfRound == 0 ? -1.0 : 1.0) * WithdrawRadius,
        //                internalProcessor.RotateByTcpZAxis(Math.PI / 2.0 * (int)ifCheckRightGalactophore, nippleTcpPostion));
        //            internalProcessor.SendURCommanderMoveL(firstIterationBeginPositionBeforeSink, fastMoveAccelerationL, fastMoveSpeedL);
        //            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Move to the lift position over the begin point of the first iteration.");

        //            Thread.Sleep(800);
        //            if (!JudgeIfMotionCanBeContinued()) return;
        //            while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
        //            {
        //                Thread.Sleep(200);
        //                if (!JudgeIfMotionCanBeContinued()) return;
        //            }
        //            if (!JudgeIfMotionCanBeContinued()) return;
        //            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Arrive at the lift position over the begin point of the first iteration.");

        //            // 2. 执行扫查力度校验
        //            DetectingIntensity preservedDetectingForceDegree = detectingForceDegree;

        //            detectingForceDegree = ForceDegreeNeed;
        //            CalculateDetectingBasicPreservedForce();
        //            internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin(NonServoFindForceTranslation.NonServoDirectionAtTcp.PositiveZ,
        //                                                                                                                                                     internalProcessor.PositionsTcpActual, checkForceSpeed, checkForceAcceleration, detectingBasicPreservedForce);

        //            Thread.Sleep(800);
        //            if (!JudgeIfMotionCanBeContinued()) return;
        //            while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
        //            {
        //                Thread.Sleep(200);
        //                if (!JudgeIfMotionCanBeContinued()) return;
        //            }
        //            if (!JudgeIfMotionCanBeContinued()) return;

        //            // 3. 提供记录力度校验下降距离 精确到mm
        //            detectingForceDegree = preservedDetectingForceDegree;

        //            Thread.Sleep(100);
        //            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Scan force is checked.");
        //        }));
        //    }
        //    else return null;
        //}

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
                        //routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[2] += detectingStopDistance; 
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
                        //routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[2] += detectingStopDistance;
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
                        //routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[2] += detectingStopDistance; 
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
                        //routeTransitPosition[2] -= detectingStopDistance;
                        routeTransitPosition[2] += detectingStopDistance; 
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
