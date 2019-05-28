using System;
using System.Collections.Generic;
using System.Reflection;
using URCommunication;
using MathFunction;
using LogPrinter;

namespace URServo
{
    /// <summary>
    /// 伺服切向运动模块
    /// </summary>
    public class ServoTangentialTranslation : ServoMotionBase
    {
        #region 枚举
        /// <summary>
        /// Tcp坐标系中的运动方向
        /// </summary>
        public enum ServoDirectionAtTcp : byte
        {
            PositiveX = 0,
            NegativeX,
            PositiveY,
            NegativeY,
            PositiveZ,
            NegativeZ
        }

        /// <summary>
        /// 停止条件
        /// </summary>
        public enum ServoStopMode : byte
        {
            DistanceCondition = 0,
            RecurrentCondition
        }
        #endregion

        #region 字段
        protected ServoDirectionAtTcp servoMotionMovingDirectionAtTcp = ServoDirectionAtTcp.NegativeY; // 相对Tcp坐标系的既定运动方向
        protected ServoDirectionAtTcp servoMotionDetectingDirectionAtTcp = ServoDirectionAtTcp.NegativeZ; // 相对Tcp坐标系的既定力保持方向
        protected double[] servoMotionMovingDirectionArrayAtTcp = new double[3]; // 相对Tcp坐标系的既定运动方向数组
        protected double[] servoMotionDetectingDirectionArrayAtTcp = new double[3]; // 相对Tcp坐标系的既定力保持方向数组
        protected double[] servoMotionVibratingDirectionArrayAtTcp = new double[3]; // 相对Tcp坐标系的既定摆动方向数组
        protected double[] servoMotionInitialMovingDirectionArrayAtBase = new double[3]; // 相对Base坐标系的既定运动方向数组
        protected double[] servoMotionInitialDetectingDirectionArrayAtBase = new double[3]; // 相对Base坐标系的既定力保持方向数组
        protected double[] servoMotionInitialVibratingDirectionArrayAtBase = new double[3]; // 相对Base坐标系的既定摆动方向数组

        protected bool servoMotionIfAttitudeChange = false; // 是否改变姿态
        protected double servoMotionUpBoundAngle = 0.0; // 姿态相对改变的上限角度

        protected ServoDirectionAtTcp servoMotionMainStopDirectionAtTcp = ServoDirectionAtTcp.NegativeY; // 相对Tcp坐标系的既定主运动停止方向
        protected ServoDirectionAtTcp servoMotionSubStopDirectionAtTcp = ServoDirectionAtTcp.NegativeZ; // 相对Tcp坐标系的既定副运动停止方向
        protected double[] servoMotionMainStopDirectionArrayAtBase = new double[3]; // 相对Base坐标系的既定主运动停止方向数组
        protected double[] servoMotionSubStopDirectionArrayAtBase = new double[3]; // 相对Base坐标系的既定副运动停止方向数组
        protected double servoMotionMainStopDirectionStopDistance = 0.0; // 在主运动停止方向上的终止距离
        protected double servoMotionSubStopDirectionStopDistance = 0.0; // 在副运动停止方向上的终止距离
        protected const int servoMotionSubStopDirectionRecurrentCheckStartRound = servoMotionInitialRound + 50;  // 在副运动停止方向上的回环检查开始轮数
        protected double servoMotionSubStopDirectionRecurrentCheckStopDistance = 0.0; // 在副运动停止方向上的回环检查的终止距离
        protected int servoMotionSubStopDirectionRecurrentCheckSign = 0; // 在副运动停止方向上的回环检查的方向符号
        protected ServoStopMode servoMotionActiveDistanceOrRecurrentCondition = ServoStopMode.DistanceCondition; // 采用停止距离条件还是回环停止条件

        protected double servoMotionInitialCoordinateAtMainStopDirection = 0.0; // 在主运动停止方向上的初始长度坐标
        protected double servoMotionInitialCoordinateAtSubStopDirection = 0.0; // 在副运动停止方向上的初始长度坐标
        protected int servoMotionAngleCalculateNeedLength = 0; // 姿态角计算所需的点数
        protected int servoMotionAngleCalculateNeedCount = 0; // 姿态角计算所需点数的计数
        protected double servoMotionRealCoordinateAtMainStopDirection = 0.0; // 在主运动停止方向上的实时长度坐标
        protected double servoMotionRealCoordinateAtSubStopDirection = 0.0; // 在副运动停止方向上的实时长度坐标

        protected double servoMotionSquareSumAtMainStopDirection = 0; // 在主运动停止方向上的平方和
        protected double servoMotionSquareSumAtSubStopDirection = 0; // 在副运动停止方向上的平方和
        protected double servoMotionCrossSumAtMainAndSubStopDirection = 0; // 在主运动停止方向和副运动停止方向上的交叉乘积求和
        protected List<double> servoMotionSquareAtMainStopDirection = new List<double>(); // 在主运动停止方向上的平方列表
        protected List<double> servoMotionSquareAtSubStopDirection = new List<double>(); // 在副运动停止方向上的平方列表
        protected List<double> servoMotionCrossAtMainAndSubStopDirection = new List<double>(); // 在主运动停止方向和副运动停止方向上的交叉乘积列表
        protected int servoMotionNextAnglePredictNeedLength = 0; // 下一个姿态角预测所需的姿态角个数
        protected int servoMotionNextAnglePredictNeedCount = 0; // 下一个姿态角预测所需的姿态角个数的计数
        protected List<double> servoMotionNextAnglePredictNeedList = new List<double>(); // 下一个姿态角预测所需的姿态角列表
        protected bool servoMotionNextAnglePredictApproachMax = false; // 下一个姿态角预测已经到达最大值
        protected int servoMotionNextAnglePredictExtensionLength = 0; // 下一个姿态角预测外延的周期数

        protected bool servoMotionPreservedForceChange = false; // 保持力是否按需要作小范围变化
        protected double servoMotionPreservedForceMaxRatio = 0.0; // 最大保持力放大倍率
        protected double servoMotionPreservedForceSwitchAngle = 0.0; // 保持力变化开关姿态角度
        protected double servoMotionPreservedForceDecayAngle = 0.0; // 保持力变化完全衰减偏置角度
        protected double servoMotionPreservedForceForChange = 0.0; // 为了变化设置的保持力大小

        protected double servoMotionMovingPeriodicalIncrement = 0.0; // 既定运动每周期增量
        protected double servoMotionDetectingPeriodicalMaxIncrement = 0.0; // 力保持方向每周期最大增量
        protected double servoMotionDetectingPeriodicalMinIncrement = 0.0; // 力保持方向每周期最小增量
        protected double servoMotionDetectingMaxAvailableForce = 0.0; // 力保持方向可接受的最大力值
        protected double servoMotionDetectingMinAvailableForce = 0.0; // 力保持方向可接受的最小力值
        protected double servoMotionVibratingPeriodicalMaxIncrement = 0.0; // 摆动方向每周期最大弧度增量

        protected double servoMotionVibratingCurrentAngle = 0.0; // 当前姿态相对改变角度

        protected const double servoMotionPredictTrimmingStartAngle = 0.1745; // 预测值修剪开始角度
        protected double servoMotionLastOriginalPredictAngle = 0.0; // 上一周期的原始预测值
        protected bool servoMotionPredictTrimmingBegin = false; // 预测值修剪过程是否开始
        protected double servoMotionLastTrimmedPredictAngle = 0.0; // 上一周期修剪好的预测值
        protected const double servoMotionTrimmingUpAngle = 0.0873; // 预测值修剪逼近上极限角度
        protected const double servoMotionTrimmingDownAngle = 0.0349; // 预测值修剪逼近下极限角度
        protected const double servoMotionTrimmingDifferenceTimes = 1.15; // 预测值修剪角度限制倍数
       
        protected List<double[]> servoMotionRecordDatas = new List<double[]>(15000); // 运动过程数据记录
        #endregion

        #region 属性
        /// <summary>
        /// 运动过程数据记录，记录在内存中，直接返回引用，使用请当心
        /// </summary>
        public List<double[]> ServoMotionRecordDatas
        {
            get { return servoMotionRecordDatas; }
        }

        /// <summary>
        /// 伺服运动模块标志属性
        /// </summary>
        protected override double ServoMotionFlag
        {
            get { return (double)ServoMotionModuleFlag.TangentialTranslation; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoTangentialTranslation(URDataProcessor Processor, bool Port30004Used)
            : base(Processor, Port30004Used) { }

        /// <summary>
        /// 伺服运动模块设置并开始
        /// </summary>
        /// <param name="MoveDirectionAtTcp">相对Tcp坐标系的既定运动方向</param>
        /// <param name="DetectDirectionAtTcp">相对Tcp坐标系的既定力保持方向</param>
        /// <param name="tcpCurrentPosition">实时Tcp坐标</param>
        /// <param name="MainStopDirectionAtTcp">相对Tcp坐标系的既定主运动停止方向</param>
        /// <param name="SubStopDirectionAtTcp">相对Tcp坐标系的既定副运动停止方向</param>
        /// <param name="MainStopDirectionStopDistance">在主运动方向上的终止距离</param>
        /// <param name="SubStopDirectionStopDistance">在副运动方向上的终止距离</param>
        /// <param name="SubStopDirectionRecurrentStopDistance">在副运动方向上的回环检查的终止距离</param>
        /// <param name="StopWay">使用的终止条件</param>
        /// <param name="ForwardSpeed">在运动方向上的速度</param>
        /// <param name="ForceMaxSpeed">在力保持方向上的最大速度</param>
        /// <param name="ForceMinSpeed">在力保持方向上的最小速度</param>
        /// <param name="ForceMaxValue">在力保持方向上的最大可接受力值</param>
        /// <param name="ForceMinValue">在力保持方向上的最小可接受力值</param>
        /// <param name="VibrateMaxValue">在摆动方向上的最大速度</param>
        /// <param name="IfRotate">是否改变姿态</param>
        /// <param name="UpAngle">姿态改变角度上限</param>
        /// <param name="PointNum">计算一个姿态角需要的数据点数量</param>
        /// <param name="AngleNum">预测下一个姿态角需要的姿态角数量</param>
        /// <param name="ExtensionNum">预测的下一个姿态角外延的周期数量</param>
        /// <param name="IfPreservedForceChange">是否打开保持力变化</param>
        /// <param name="PreservedForceMaxRatio">保持力变化的最大倍率</param>
        /// <param name="PreservedForceSwitchAngle">保持力变化的开关姿态角</param>
        /// <param name="PreservedForceDeltaAngle">保持力变化的全衰减角</param>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(ServoDirectionAtTcp MoveDirectionAtTcp,
                                                                    ServoDirectionAtTcp DetectDirectionAtTcp,
                                                                    double[] tcpCurrentPosition,
                                                                    ServoDirectionAtTcp MainStopDirectionAtTcp,
                                                                    ServoDirectionAtTcp SubStopDirectionAtTcp,
                                                                    double MainStopDirectionStopDistance,
                                                                    double SubStopDirectionStopDistance,
                                                                    double SubStopDirectionRecurrentStopDistance,
                                                                    ServoStopMode StopWay,
                                                                    double ForwardSpeed,
                                                                    double ForceMaxSpeed,
                                                                    double ForceMinSpeed,
                                                                    double ForceMaxValue,
                                                                    double ForceMinValue,
                                                                    double VibrateMaxValue,
                                                                    bool IfRotate,
                                                                    double UpAngle,
                                                                    int PointNum = 125,
                                                                    int AngleNum = 75,
                                                                    int ExtensionNum = 12,
                                                                    bool IfPreservedForceChange = true,
                                                                    double PreservedForceMaxRatio = 1.5,
                                                                    double PreservedForceSwitchAngle = 0.5236,
                                                                    double PreservedForceDeltaAngle = 0.0698,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设定运动和力保持方向
            servoMotionMovingDirectionAtTcp = MoveDirectionAtTcp;
            servoMotionDetectingDirectionAtTcp = DetectDirectionAtTcp;

            // 获得运动和力保持方向的Tcp系和Base系表示
            double[] tcpToBase = URMath.ReverseReferenceRelationship(tcpCurrentPosition);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });
            switch (servoMotionMovingDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { 1.0, 0.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.NegativeX:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { -1.0, 0.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.PositiveY:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { 0.0, 1.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.NegativeY:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { 0.0, -1.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { 0.0, 0.0, 1.0 };
                    break;
                case ServoDirectionAtTcp.NegativeZ:
                    servoMotionMovingDirectionArrayAtTcp = new double[] { 0.0, 0.0, -1.0 };
                    break;
                default:
                    break;
            }
            servoMotionInitialMovingDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionMovingDirectionArrayAtTcp, qTcpToBase);
            switch (servoMotionDetectingDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                case ServoDirectionAtTcp.NegativeX:
                    servoMotionDetectingDirectionArrayAtTcp = new double[] { 1.0, 0.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.PositiveY:
                case ServoDirectionAtTcp.NegativeY:
                    servoMotionDetectingDirectionArrayAtTcp = new double[] { 0.0, 1.0, 0.0 };
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                case ServoDirectionAtTcp.NegativeZ:
                    servoMotionDetectingDirectionArrayAtTcp = new double[] { 0.0, 0.0, 1.0 };
                    break;
                default:
                    break;
            }
            servoMotionInitialDetectingDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionDetectingDirectionArrayAtTcp, qTcpToBase);

            // 设定摆动方向
            servoMotionVibratingDirectionArrayAtTcp = URMath.VectorCrossMultiply(servoMotionMovingDirectionArrayAtTcp, servoMotionDetectingDirectionArrayAtTcp);
            servoMotionInitialVibratingDirectionArrayAtBase = URMath.VectorCrossMultiply(servoMotionInitialMovingDirectionArrayAtBase, servoMotionInitialDetectingDirectionArrayAtBase);

            // 设定主运动停止方向和副运动停止方向
            servoMotionMainStopDirectionAtTcp = MainStopDirectionAtTcp;
            servoMotionSubStopDirectionAtTcp = SubStopDirectionAtTcp;

            // 获得主运动和副运动停止方向的Base系表示
            switch (servoMotionMainStopDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeX:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { -1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveY:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeY:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, -1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, 1.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeZ:
                    servoMotionMainStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, -1.0 }, qTcpToBase);
                    break;
                default:
                    break;
            }
            switch (servoMotionSubStopDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeX:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { -1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveY:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeY:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, -1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, 1.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeZ:
                    servoMotionSubStopDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, -1.0 }, qTcpToBase);
                    break;
                default:
                    break;
            }

            // 设定主运动和副运动停止方向的最大行进距离、回环停止距离以及停止条件的选择
            servoMotionMainStopDirectionStopDistance = MainStopDirectionStopDistance;
            servoMotionSubStopDirectionStopDistance = SubStopDirectionStopDistance;
            servoMotionSubStopDirectionRecurrentCheckStopDistance = SubStopDirectionRecurrentStopDistance;
            servoMotionActiveDistanceOrRecurrentCondition = StopWay;

            // 设定运动速度
            servoMotionMovingPeriodicalIncrement = ForwardSpeed;

            // 设定力保持方向速度和力限制
            servoMotionDetectingPeriodicalMaxIncrement = ForceMaxSpeed;
            servoMotionDetectingPeriodicalMinIncrement = ForceMinSpeed;
            servoMotionDetectingMaxAvailableForce = ForceMaxValue;
            servoMotionDetectingMinAvailableForce = ForceMinValue;

            // 设定摆动方向的速度限制
            servoMotionVibratingPeriodicalMaxIncrement = VibrateMaxValue;

            // 设定姿态是否更改，以及更改的上下限
            servoMotionIfAttitudeChange = IfRotate;
            servoMotionUpBoundAngle = UpAngle;

            // 设置姿态计算所需参数个数
            servoMotionAngleCalculateNeedLength = PointNum;
            servoMotionNextAnglePredictNeedLength = AngleNum;
            servoMotionNextAnglePredictExtensionLength = ExtensionNum;

            // 设置保持力变化是否打开以及其所需的参数
            servoMotionPreservedForceChange = IfPreservedForceChange;
            servoMotionPreservedForceMaxRatio = PreservedForceMaxRatio;
            servoMotionPreservedForceSwitchAngle = PreservedForceSwitchAngle;
            servoMotionPreservedForceDecayAngle = PreservedForceDeltaAngle;

            // 重置角度计算所需参数列表
            servoMotionSquareAtMainStopDirection = new List<double>(servoMotionAngleCalculateNeedLength);
            servoMotionSquareAtSubStopDirection = new List<double>(servoMotionAngleCalculateNeedLength);
            servoMotionCrossAtMainAndSubStopDirection = new List<double>(servoMotionAngleCalculateNeedLength);
            servoMotionSquareAtMainStopDirection.Clear();
            servoMotionSquareAtSubStopDirection.Clear();
            servoMotionCrossAtMainAndSubStopDirection.Clear();

            // 重置姿态计算所需角度列表
            servoMotionNextAnglePredictNeedList = new List<double>(servoMotionNextAnglePredictNeedLength);
            servoMotionNextAnglePredictNeedList.Clear();

            // 设置初始位置在两个停止方向上的初值
            servoMotionInitialCoordinateAtMainStopDirection = URMath.VectorDotMultiply(new double[] { tcpCurrentPosition[0], tcpCurrentPosition[1], tcpCurrentPosition[2] }, servoMotionMainStopDirectionArrayAtBase);
            servoMotionInitialCoordinateAtSubStopDirection = URMath.VectorDotMultiply(new double[] { tcpCurrentPosition[0], tcpCurrentPosition[1], tcpCurrentPosition[2] }, servoMotionSubStopDirectionArrayAtBase);

            // 初始化角度预测计数器
            servoMotionAngleCalculateNeedCount = 0;
            servoMotionNextAnglePredictNeedCount = 0;

            // 初始化角度预测所需求和量
            servoMotionSquareSumAtMainStopDirection = 0;
            servoMotionSquareSumAtSubStopDirection = 0;
            servoMotionCrossSumAtMainAndSubStopDirection = 0;

            // 初始化力保持的值
            servoMotionPreservedForce[0] = 0.0;
            servoMotionPreservedForceForChange = 0.0;

            // 初始化姿态实时相对角度
            servoMotionVibratingCurrentAngle = 0.0;

            // 初始化姿态角限幅状态值
            servoMotionNextAnglePredictApproachMax = false;

            // 初始化预测修剪标志位
            servoMotionPredictTrimmingBegin = false;

            // 设置伺服参数并重写下位机控制程序
            SetServoMotionParameters(ControlPeriod, LookAheadTime, Gain);
            internalProcessor.WriteStringToControlCode(ControlPeriod, LookAheadTime, Gain);

            // 打开伺服模块时对一般逻辑的处理
            internalProcessor.ServoSwitchMode(true);

            // 开始本伺服模式
            ServoMotionBegin();
        }

        /// <summary>
        /// 伺服运动的准备工作，包括可能的伺服数据交换设置，保持力记录，初始位置记录和发送，并开始下位机程序
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        protected override void ServoMotionGetReady(double[] tcpRealPosition, double[] referenceForce)
        {
            switch (servoMotionOpenRound)
            {
                // 每个周期都要采集保持力，最后一个周期求平均
                case 0: // 第一个周期 传参端口设置
                    if (ifPort30004Used)
                    {
                        internalProcessor.SendURServorInputSetup();
                    }
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);
                    break;
                case 1:
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);
                    break;
                case 2: // 第三个周期 清空数据记录
                    servoMotionRecordDatas.Clear();
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);
                    break;
                case 3: // 第四个周期 获得Tcp位置并下发传参端口作为初始值
                    servoMotionBeginTcpPosition = (double[])tcpRealPosition.Clone();
                    if (ifPort30004Used)
                    {
                        internalProcessor.SendURServorInputDatas(servoMotionBeginTcpPosition);
                    }
                    else
                    {
                        internalProcessor.SendURModbusInputDatas(servoMotionBeginTcpPosition);
                    }
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);
                    break;
                case 4: // 第五个周期 下达下位机指令并运行
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);
                    servoMotionPreservedForce[0] /= servoMotionInitialRound;
                    servoMotionPreservedForceForChange = servoMotionPreservedForce[0];

                    servoMotionRecordDatas.Add(ServoMotionOutputConfiguration());

                    internalProcessor.SendURCommanderControllerCode();
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// 伺服运动模块是否到达终止条件
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <returns>返回是否终止</returns>
        protected override bool ServoMotionIfFinished(double[] tcpRealPosition)
        {
            // 继承基类判断
            if (base.ServoMotionIfFinished(tcpRealPosition))
            {
                return true;
            }

            // 主运动方向终止
            double[] moveArray = new double[] { tcpRealPosition[0] - servoMotionBeginTcpPosition[0], tcpRealPosition[1] - servoMotionBeginTcpPosition[1], tcpRealPosition[2] - servoMotionBeginTcpPosition[2] };
            if (URMath.VectorDotMultiply(moveArray, servoMotionMainStopDirectionArrayAtBase) > servoMotionMainStopDirectionStopDistance)
            {
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Main stop direction limitation reached.");
                return true;
            }

            if (servoMotionActiveDistanceOrRecurrentCondition == ServoStopMode.DistanceCondition)
            {
                // 副运动方向距离终止
                if (Math.Abs(URMath.VectorDotMultiply(moveArray, servoMotionSubStopDirectionArrayAtBase)) > servoMotionSubStopDirectionStopDistance)
                {
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Sub stop direction limitation reached.");
                    return true;
                }
            }
            else
            {
                // 副运动方向回环终止
                if (servoMotionOpenRound > servoMotionSubStopDirectionRecurrentCheckStartRound)
                {
                    if (servoMotionSubStopDirectionRecurrentCheckSign * URMath.VectorDotMultiply(moveArray, servoMotionSubStopDirectionArrayAtBase) < servoMotionSubStopDirectionRecurrentCheckStopDistance)
                    {
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Sub stop direction recurrent reached.");
                        return true;
                    }
                }
                else if (servoMotionOpenRound == servoMotionSubStopDirectionRecurrentCheckStartRound)
                {
                    servoMotionSubStopDirectionRecurrentCheckSign = Math.Sign(URMath.VectorDotMultiply(moveArray, servoMotionSubStopDirectionArrayAtBase));
                    servoMotionOpenRound++;
                }
                else
                {
                    servoMotionOpenRound++;
                }
            }

            return false;
        }

        /// <summary>
        /// 伺服运动模块中计算下一周期的Tcp位置
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        /// <returns>返回下一周期的Tcp位置</returns>
        protected override double[] ServoMotionNextTcpPosition(double[] tcpRealPosition, double[] referenceForce, double[] moreInfo = null)
        {
            double[] nextTcpPosition = (double[])tcpRealPosition.Clone();

            // 暂停运动
            if (servoMotionActivePause)
            {
                return nextTcpPosition;
            }

            // 姿态矫正开启并且保持力变化开启时，首先进行姿态角的预测，并按照姿态角预测结果对保持力进行适当变化
            double predictAngleBeforeTrimming = 0.0;
            double predictAngleBeforeLimiting = 0.0;
            double predictAngle = 0.0;
            if (servoMotionIfAttitudeChange && servoMotionPreservedForceChange)
            {
                predictAngle = ServoMotionWorkToPredictNextAngleBeforeApproachMax(tcpRealPosition, ref predictAngleBeforeTrimming, ref predictAngleBeforeLimiting);

                ServoMotionChangePreservedForceByPredictAttitude(predictAngle);
            }

            // 无论是否打开姿态变化，只要极限姿态角不到，就进行一般位置修正
            if (!servoMotionNextAnglePredictApproachMax)
            {
                // 获得实时运动方向在Base系的表示
                double[] tcpToBase = URMath.ReverseReferenceRelationship(tcpRealPosition);
                Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });
                double[] movingDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionMovingDirectionArrayAtTcp, qTcpToBase);
                double[] detectingDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionDetectingDirectionArrayAtTcp, qTcpToBase);

                // 运动方向上加一定的增量
                for (int k = 0; k < 3; k++)
                {
                    nextTcpPosition[k] += movingDirectionAtBase[k] * servoMotionMovingPeriodicalIncrement;
                }

                // 力保持方向上加一定的增量
                double differenceForce = URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp) - servoMotionPreservedForceForChange;
                double forceDirectionIncrement = 0.0;
                if (Math.Abs(differenceForce) <= servoMotionDetectingMinAvailableForce)
                {
                    forceDirectionIncrement = 0.0;
                }
                else if (Math.Abs(differenceForce) >= servoMotionDetectingMaxAvailableForce)
                {
                    forceDirectionIncrement = Math.Sign(differenceForce) * servoMotionDetectingPeriodicalMaxIncrement;
                }
                else
                {
                    forceDirectionIncrement = Math.Sign(differenceForce) * ((Math.Abs(differenceForce) - servoMotionDetectingMinAvailableForce) / (servoMotionDetectingMaxAvailableForce - servoMotionDetectingMinAvailableForce) * (servoMotionDetectingPeriodicalMaxIncrement - servoMotionDetectingPeriodicalMinIncrement) + servoMotionDetectingPeriodicalMinIncrement);
                }
                for (int k = 0; k < 3; k++)
                {
                    nextTcpPosition[k] += detectingDirectionAtBase[k] * forceDirectionIncrement;
                }
            }

            // 一般姿态修正
            if (servoMotionIfAttitudeChange)
            {
                // 保持力变化没有开启，对姿态角进行预测
                if (!servoMotionPreservedForceChange)
                {
                    predictAngle = ServoMotionWorkToPredictNextAngleBeforeApproachMax(tcpRealPosition, ref predictAngleBeforeTrimming, ref predictAngleBeforeLimiting);
                }

                // 达到极限姿态角时，执行特殊的位置修正
                if (servoMotionNextAnglePredictApproachMax)
                {
                    // 获得实时运动方向在Base系的表示
                    double[] tcpToBase = URMath.ReverseReferenceRelationship(tcpRealPosition);
                    Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });
                    double[] movingDirectionAtBaseForPlan = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionMovingDirectionArrayAtTcp, qTcpToBase);
                    double[] detectingDirectionAtBaseForPlan = URMath.FindDirectionToSecondReferenceFromFirstReference(servoMotionDetectingDirectionArrayAtTcp, qTcpToBase);

                    // 获得预测角度下的运动和力保持方向
                    double extensionAngle = predictAngle - servoMotionUpBoundAngle;
                    double extensionAngleCos = Math.Cos(extensionAngle);
                    double extensionAngleSin = Math.Sin(extensionAngle);
                    double[] movingDirectionAtBaseForPredict = URMath.VectorGeneratedFromBaseArray(extensionAngleCos, extensionAngleSin, movingDirectionAtBaseForPlan, detectingDirectionAtBaseForPlan);
                    double[] detectingDirectionAtBaseForPredict = URMath.VectorGeneratedFromBaseArray(-extensionAngleSin, extensionAngleCos, movingDirectionAtBaseForPlan, detectingDirectionAtBaseForPlan);
                    double[] detectingDirectionAtTcpForPredict = URMath.VectorGeneratedFromBaseArray(-extensionAngleSin, extensionAngleCos, servoMotionMovingDirectionArrayAtTcp, servoMotionDetectingDirectionArrayAtTcp);

                    // 运动方向上加一定的增量
                    for (int k = 0; k < 3; k++)
                    {
                        nextTcpPosition[k] += movingDirectionAtBaseForPredict[k] * servoMotionMovingPeriodicalIncrement;
                    }

                    // 力保持方向上加一定的增量
                    double differenceForce = URMath.VectorDotMultiply(referenceForce, detectingDirectionAtTcpForPredict) - servoMotionPreservedForceForChange;
                    double forceDirectionIncrement = 0.0;
                    if (Math.Abs(differenceForce) <= servoMotionDetectingMinAvailableForce)
                    {
                        forceDirectionIncrement = 0.0;
                    }
                    else if (Math.Abs(differenceForce) >= servoMotionDetectingMaxAvailableForce)
                    {
                        forceDirectionIncrement = Math.Sign(differenceForce) * servoMotionDetectingPeriodicalMaxIncrement;
                    }
                    else
                    {
                        forceDirectionIncrement = Math.Sign(differenceForce) * ((Math.Abs(differenceForce) - servoMotionDetectingMinAvailableForce) / (servoMotionDetectingMaxAvailableForce - servoMotionDetectingMinAvailableForce) * (servoMotionDetectingPeriodicalMaxIncrement - servoMotionDetectingPeriodicalMinIncrement) + servoMotionDetectingPeriodicalMinIncrement);
                    }
                    for (int k = 0; k < 3; k++)
                    {
                        nextTcpPosition[k] += detectingDirectionAtBaseForPredict[k] * forceDirectionIncrement;
                    }

                    double[] nextPosture = URMath.Quatnum2AxisAngle(
                                                          URMath.QuatnumRotate(new Quatnum[] { 
                                                                                                 URMath.AxisAngle2Quatnum(new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] }), 
                                                                                                 URMath.AxisAngle2Quatnum(new double[] { servoMotionUpBoundAngle * servoMotionInitialVibratingDirectionArrayAtBase[0], servoMotionUpBoundAngle * servoMotionInitialVibratingDirectionArrayAtBase[1], servoMotionUpBoundAngle * servoMotionInitialVibratingDirectionArrayAtBase[2] }) }));

                    nextTcpPosition[3] = nextPosture[0];
                    nextTcpPosition[4] = nextPosture[1];
                    nextTcpPosition[5] = nextPosture[2];
                }
                else // 不到极限姿态角时，预测姿态角为零则不修正姿态
                {
                    if (predictAngle != 0.0)
                    {
                        double[] nextPosture = URMath.Quatnum2AxisAngle(
                                                              URMath.QuatnumRotate(new Quatnum[] { 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] }), 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[0], predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[1], predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[2] }) }));

                        nextTcpPosition[3] = nextPosture[0];
                        nextTcpPosition[4] = nextPosture[1];
                        nextTcpPosition[5] = nextPosture[2];
                    }
                }
            }

            // 记录数据
            servoMotionRecordDatas.Add(new double[] { tcpRealPosition[0], 
                                                                                    tcpRealPosition[1], 
                                                                                    tcpRealPosition[2], 
                                                                                    tcpRealPosition[3], 
                                                                                    tcpRealPosition[4], 
                                                                                    tcpRealPosition[5], 
                                                                                    referenceForce[0], 
                                                                                    referenceForce[1], 
                                                                                    referenceForce[2], 
                                                                                    referenceForce[3], 
                                                                                    referenceForce[4], 
                                                                                    referenceForce[5], 
                                                                                    servoMotionPreservedForceForChange,
                                                                                    predictAngleBeforeTrimming,
                                                                                    predictAngleBeforeLimiting,
                                                                                    predictAngle });

            return nextTcpPosition;
        }

        /// <summary>
        /// 计算累计数据对应的姿态角度，即二阶方阵最大特征值的特征向量的反正切
        /// 假设 a = Σ x^2，b = Σ y^2，c = Σ xy
        ///                         __                    ___________________     __
        ///                         |       (c - a) + _| (c - a)^2 + 4b^2        |
        /// angle = arctan | ----------------------------------------- |
        ///                         |__                        2b                        __|
        /// </summary>
        /// <returns>返回计算得到的姿态角度</returns>
        protected double ServoMotionCalculateAngleFromPointList()
        {
            return Math.Atan(
                                        (
                                         (servoMotionSquareSumAtSubStopDirection - servoMotionSquareSumAtMainStopDirection) +
                                         Math.Sqrt(
                                                         Math.Pow((servoMotionSquareSumAtSubStopDirection - servoMotionSquareSumAtMainStopDirection), 2) +
                                                         4.0 * Math.Pow(servoMotionCrossSumAtMainAndSubStopDirection, 2)
                                                         )
                                         ) /
                                         (2.0 * servoMotionCrossSumAtMainAndSubStopDirection)
                                        );
        }

        /// <summary>
        /// 预测下一个姿态角
        /// </summary>
        /// <returns>返回预测值</returns>
        protected double ServoMotionPredictNextAngle()
        {
            return servoMotionNextAnglePredictNeedList[servoMotionNextAnglePredictNeedLength - 1] +
                      (servoMotionAngleCalculateNeedLength / 2.0 + servoMotionNextAnglePredictExtensionLength) / servoMotionNextAnglePredictNeedLength *
                      (servoMotionNextAnglePredictNeedList[servoMotionNextAnglePredictNeedLength - 1] - servoMotionNextAnglePredictNeedList[0]);
        }

        /// <summary>
        /// 在预测的姿态角未达到最大值前对其进行预测
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="predictAngleBeforeTrimming">修剪前预测的姿态角</param>
        /// <param name="predictAngleBeforeLimiting">约束前预测的姿态角</param>
        /// <returns>返回预测的姿态角</returns>
        protected double ServoMotionWorkToPredictNextAngleBeforeApproachMax(double[] tcpRealPosition, ref double predictAngleBeforeTrimming, ref double predictAngleBeforeLimiting)
        {
            double predictAngle = 0.0;

            // 获得当前位置在运动停止方向坐标系中的坐标，以初始点为原点
            double xOfStopCoordinate = URMath.VectorDotMultiply(new double[] { tcpRealPosition[0], tcpRealPosition[1], tcpRealPosition[2] }, servoMotionMainStopDirectionArrayAtBase) - servoMotionInitialCoordinateAtMainStopDirection;
            double yOfStopCoordinate = URMath.VectorDotMultiply(new double[] { tcpRealPosition[0], tcpRealPosition[1], tcpRealPosition[2] }, servoMotionSubStopDirectionArrayAtBase) - servoMotionInitialCoordinateAtSubStopDirection;

            if (servoMotionAngleCalculateNeedCount < servoMotionAngleCalculateNeedLength - 1)
            {
                // 将累计值计入列表
                servoMotionSquareAtMainStopDirection.Add(Math.Pow(xOfStopCoordinate, 2));
                servoMotionSquareAtSubStopDirection.Add(Math.Pow(yOfStopCoordinate, 2));
                servoMotionCrossAtMainAndSubStopDirection.Add(xOfStopCoordinate * yOfStopCoordinate);

                // 累计计算方阵各元素的值
                servoMotionSquareSumAtMainStopDirection += servoMotionSquareAtMainStopDirection[servoMotionAngleCalculateNeedCount];
                servoMotionSquareSumAtSubStopDirection += servoMotionSquareAtSubStopDirection[servoMotionAngleCalculateNeedCount];
                servoMotionCrossSumAtMainAndSubStopDirection += servoMotionCrossAtMainAndSubStopDirection[servoMotionAngleCalculateNeedCount];

                servoMotionAngleCalculateNeedCount++;

                // 直接返回结果
                return predictAngle;
            }
            else if (servoMotionAngleCalculateNeedCount == servoMotionAngleCalculateNeedLength - 1)
            {
                // 将累计值计入列表
                servoMotionSquareAtMainStopDirection.Add(Math.Pow(xOfStopCoordinate, 2));
                servoMotionSquareAtSubStopDirection.Add(Math.Pow(yOfStopCoordinate, 2));
                servoMotionCrossAtMainAndSubStopDirection.Add(xOfStopCoordinate * yOfStopCoordinate);

                // 累计计算方阵各元素的值
                servoMotionSquareSumAtMainStopDirection += servoMotionSquareAtMainStopDirection[servoMotionAngleCalculateNeedCount];
                servoMotionSquareSumAtSubStopDirection += servoMotionSquareAtSubStopDirection[servoMotionAngleCalculateNeedCount];
                servoMotionCrossSumAtMainAndSubStopDirection += servoMotionCrossAtMainAndSubStopDirection[servoMotionAngleCalculateNeedCount];

                servoMotionAngleCalculateNeedCount++;

                // 计算累计数据对应的姿态角度
                double attitudeAngle = ServoMotionCalculateAngleFromPointList();

                // 将计算得出的姿态角添加到列表中
                servoMotionNextAnglePredictNeedList.Add(attitudeAngle);

                servoMotionNextAnglePredictNeedCount++;

                // 直接返回结果
                return predictAngle;
            }
            else
            {
                // 从累计值中扣除第一项的贡献
                servoMotionSquareSumAtMainStopDirection -= servoMotionSquareAtMainStopDirection[0];
                servoMotionSquareSumAtSubStopDirection -= servoMotionSquareAtSubStopDirection[0];
                servoMotionCrossSumAtMainAndSubStopDirection -= servoMotionCrossAtMainAndSubStopDirection[0];

                // 删除累计值列表的第一项
                servoMotionSquareAtMainStopDirection.RemoveAt(0);
                servoMotionSquareAtSubStopDirection.RemoveAt(0);
                servoMotionCrossAtMainAndSubStopDirection.RemoveAt(0);

                // 将累计值计入列表
                servoMotionSquareAtMainStopDirection.Add(Math.Pow(xOfStopCoordinate, 2));
                servoMotionSquareAtSubStopDirection.Add(Math.Pow(yOfStopCoordinate, 2));
                servoMotionCrossAtMainAndSubStopDirection.Add(xOfStopCoordinate * yOfStopCoordinate);

                // 累计计算方阵各元素的值
                servoMotionSquareSumAtMainStopDirection += servoMotionSquareAtMainStopDirection[servoMotionAngleCalculateNeedLength - 1];
                servoMotionSquareSumAtSubStopDirection += servoMotionSquareAtSubStopDirection[servoMotionAngleCalculateNeedLength - 1];
                servoMotionCrossSumAtMainAndSubStopDirection += servoMotionCrossAtMainAndSubStopDirection[servoMotionAngleCalculateNeedLength - 1];

                // 计算累计数据对应的姿态角度
                double attitudeAngle = ServoMotionCalculateAngleFromPointList();

                if (servoMotionNextAnglePredictNeedCount < servoMotionNextAnglePredictNeedLength - 1)
                {
                    // 将计算得出的姿态角添加到列表中
                    servoMotionNextAnglePredictNeedList.Add(attitudeAngle);

                    servoMotionNextAnglePredictNeedCount++;

                    // 直接返回结果
                    return predictAngle;
                }
                else if (servoMotionNextAnglePredictNeedCount == servoMotionNextAnglePredictNeedLength - 1)
                {
                    // 将计算得出的姿态角添加到列表中
                    servoMotionNextAnglePredictNeedList.Add(attitudeAngle);

                    servoMotionNextAnglePredictNeedCount++;

                    // 预测下一个姿态角
                    predictAngle = ServoMotionPredictNextAngle();
                }
                else
                {
                    // 如果预测值已经到达45度后出现负号的预测值，则说明跳过90度
                    if (Math.Abs(servoMotionVibratingCurrentAngle) > 0.8 && attitudeAngle < 0)
                    {
                        attitudeAngle += Math.PI;
                    }

                    // 替换姿态角记录
                    servoMotionNextAnglePredictNeedList.RemoveAt(0);
                    servoMotionNextAnglePredictNeedList.Add(attitudeAngle);

                    // 预测下一个姿态角
                    predictAngle = ServoMotionPredictNextAngle();
                }
            }

            predictAngleBeforeTrimming = predictAngle;

            // 修剪预测值
            if (Math.Sign(predictAngle) * servoMotionUpBoundAngle > 0 && Math.Abs(predictAngle) > servoMotionPredictTrimmingStartAngle)
            {
                servoMotionPredictTrimmingBegin = true;
            }
            if (servoMotionPredictTrimmingBegin)
            {
                double setDifference = servoMotionTrimmingDifferenceTimes * servoMotionVibratingPeriodicalMaxIncrement;

                double predictDifference = predictAngle - servoMotionLastOriginalPredictAngle;
                double predictDifferenceMagnitude = Math.Abs(predictDifference);
                int predictDifferenceSign = Math.Sign(predictDifference) * servoMotionUpBoundAngle > 0 ? 1 : -1;

                double tempAngle = servoMotionLastTrimmedPredictAngle + predictDifferenceSign * predictDifferenceMagnitude;
                double tempAngleMagnitude = Math.Abs(tempAngle);

                double predictTrimMagnitude = Math.Abs(predictAngle);

                if (tempAngleMagnitude < predictTrimMagnitude)
                {
                    if (predictDifferenceSign < 0)
                    {
                        double distance = Math.Abs(tempAngle - predictAngle);
                        if (distance > servoMotionTrimmingUpAngle)
                        {
                            predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference;
                        }
                        else if (distance > servoMotionTrimmingDownAngle)
                        {
                            predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference * (distance - servoMotionTrimmingDownAngle) / (servoMotionTrimmingUpAngle - servoMotionTrimmingDownAngle);
                        }
                        else
                        {
                            predictAngle = servoMotionLastTrimmedPredictAngle;
                        }
                    }
                    else if (predictDifferenceMagnitude < setDifference)
                    {
                        double distance = Math.Abs(tempAngle - predictAngle);
                        if (distance > servoMotionTrimmingUpAngle)
                        {
                            predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference;
                        }
                        else
                        {
                            predictAngle = tempAngle;
                        }
                    }
                    else
                    {
                        predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference;
                    }
                }
                else if (tempAngleMagnitude == predictTrimMagnitude)
                {
                    if (predictDifferenceSign < 0 || predictDifferenceMagnitude < setDifference)
                    {
                        predictAngle = tempAngle;
                    }
                    else
                    {
                        predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference;
                    }
                }
                else
                {
                    if (predictDifferenceSign < 0 || predictDifferenceMagnitude < setDifference)
                    {
                        predictAngle = tempAngle;
                    }
                    else
                    {
                        predictAngle = servoMotionLastTrimmedPredictAngle + Math.Sign(servoMotionUpBoundAngle) * setDifference;
                    }
                }
            }

            servoMotionLastTrimmedPredictAngle = predictAngle;
            predictAngleBeforeLimiting = predictAngle;

            double predictMagnitude = Math.Abs(predictAngle);
            int predictSign = Math.Sign(predictAngle);
            double lastpredictMagnitude = Math.Abs(servoMotionVibratingCurrentAngle);
            if (predictMagnitude < lastpredictMagnitude || predictSign * servoMotionUpBoundAngle < 0) // 预测值单向变化
            {
                predictAngle = servoMotionVibratingCurrentAngle;
            }
            else
            {
                if (predictMagnitude - lastpredictMagnitude > servoMotionVibratingPeriodicalMaxIncrement) // 预测值变化限速
                {
                    predictAngle = Math.Sign(servoMotionUpBoundAngle) * (lastpredictMagnitude + servoMotionVibratingPeriodicalMaxIncrement);
                }
                servoMotionVibratingCurrentAngle = predictAngle;
            }

            // 预测值限幅
            if (!servoMotionNextAnglePredictApproachMax && Math.Abs(predictAngle) >= Math.Abs(servoMotionUpBoundAngle))
            {
                servoMotionNextAnglePredictApproachMax = true;
            }

            // 保存原始预测值
            servoMotionLastOriginalPredictAngle = predictAngleBeforeTrimming;

            return predictAngle;
        }

        /// <summary>
        /// 根据预测姿态角改变保持力大小
        /// </summary>
        /// <param name="PredictAngle">预测的姿态角</param>
        protected void ServoMotionChangePreservedForceByPredictAttitude(double PredictAngle)
        {
            double factor = 1.0 + (servoMotionPreservedForceMaxRatio - 1.0) /
                                               (
                                                1.0 + Math.Exp(
                                                                        -5.0 / servoMotionPreservedForceDecayAngle *
                                                                         (PredictAngle - servoMotionPreservedForceSwitchAngle)
                                                                        )
                                               );
            servoMotionPreservedForceForChange = servoMotionPreservedForce[0] * factor;
        }

        /// <summary>
        /// 伺服运动模块部分配置参数输出
        /// </summary>
        /// <returns>配置参数输出</returns>
        protected override double[] ServoMotionOutputConfiguration()
        {
            return new double[]{ ServoMotionFlag,
                                              servoMotionPreservedForce[0],
                                              servoMotionMainStopDirectionStopDistance,
                                              servoMotionSubStopDirectionStopDistance,
                                              (double)servoMotionActiveDistanceOrRecurrentCondition,
                                              servoMotionSubStopDirectionRecurrentCheckStopDistance,
                                              servoMotionMovingPeriodicalIncrement,
                                              servoMotionDetectingPeriodicalMinIncrement,
                                              servoMotionDetectingPeriodicalMaxIncrement,
                                              servoMotionDetectingMinAvailableForce,
                                              servoMotionDetectingMaxAvailableForce,
                                              servoMotionVibratingPeriodicalMaxIncrement,
                                              servoMotionIfAttitudeChange ? 1.0 : 0.0,
                                              servoMotionUpBoundAngle,
                                              servoMotionAngleCalculateNeedLength,
                                              servoMotionNextAnglePredictNeedLength,
                                              servoMotionNextAnglePredictExtensionLength,
                                              servoMotionPreservedForceChange ? 1.0 : 0.0,
                                              servoMotionPreservedForceMaxRatio,
                                              servoMotionPreservedForceSwitchAngle,
                                              servoMotionPreservedForceDecayAngle };

        }
        #endregion
    }
}
