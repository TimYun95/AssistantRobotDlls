using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using URCommunication;
using MathFunction;
using LogPrinter;

namespace URServo
{
    /// <summary>
    /// 伺服切向运动模块（使用切向力）
    /// </summary>
    public class ServoTangentialTranslationWithForce : ServoMotionBase
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

        protected double servoMotionInitialAngle = 0.0; // 运动初始姿态角
        protected bool servoMotionIfFindInitialAngle = false; // 是否寻找运动初始姿态角
        protected bool servoMotionIfRecorrectRotateAngle = false; // 是否矫正姿态角
        protected double servoMotionIgnoreJudgeSpan = 0.001; // 姿态角矫正忽略长度        
        protected double servoMotionJudgeStep = 0.003; // 姿态角矫正步长
        protected double servoMotionOverlappingRate = 0.5; // 姿态角矫正采样重叠率

        protected const double servoMotionAdjustFrictionBiasIni = 0.0; // 姿态角预测摩擦力模板函数力值偏移初始值
        protected double servoMotionAdjustBias = servoMotionAdjustFrictionBiasIni; // 姿态角预测摩擦力模板函数力值偏移
        protected double servoMotionInstantBias = servoMotionAdjustFrictionBiasIni; // 姿态角预测摩擦力模板函数瞬时力值偏移

        protected double servoMotionFromFrictionToAngle = 0.015; // 多少摩擦力对应多少角度偏转
        protected double servoMotionFromAngleToFriction = 20.0; // 多少角度误差对应多少目标摩擦力修正量
        protected const double servoMotionMaxFrictionForceBias = 1.0; // 最大目标摩擦力修正量

        protected bool servoMotionVibrateAngleModifyBegin = false; // 是否开始矫正姿态角
        protected bool servoMotionVibrateAngleIgnoreRegionBeyond = false; // 是否越过姿态角矫正忽略区域
        protected List<double> servoMotionMovingDirectionCordinates = new List<double>(15000); // 运动方向坐标
        protected List<double> servoMotionDetectingForceDirectionCordinates = new List<double>(15000); // 力探测方向坐标
        protected List<double> servoMotionVibratingAngleCordinates = new List<double>(15000); // 摆动方向坐标

        protected List<double> servoMotionVibratingAnglePredict = new List<double>(15000); // 摆动方向角度预测
        protected List<double> servoMotionVibratingAnglePredictAfterFilter = new List<double>(15000); // 摆动方向角度经过滤波后的预测
        protected List<double> servoMotionVibratingAnglePredictAfterAccum = new List<double>(15000); // 摆动方向角度经过滤波和基波修正后的预测

        protected int servoMotionLastIndex = 0; // 上一次预测的索引
        protected double servoMotionMeanAngleChange = 0.0; // 平均预测角度差
        protected const double servoMotionModifyAngleTrustCoeff = 0.5; // 角度矫正采信程度

        protected double servoMotionAccmuErr = 0.0; // 姿态角基波预测
        protected const double servoMotionAlpha = 0.4; // 姿态角滤波比例
        protected const double servoMotionMaxAngleDiff = 3.0 / 180.0 * Math.PI; // 姿态角变化模板最大值

        protected const double servoMotionDetectForceBackDistance = 0.0005; // 力探测方向回退距离

        protected double servoMotionMovingPeriodicalIncrement = 0.0; // 既定运动每周期增量
        protected double servoMotionDetectingPeriodicalMaxIncrement = 0.0; // 力保持方向每周期最大增量
        protected double servoMotionDetectingPeriodicalMinIncrement = 0.0; // 力保持方向每周期最小增量
        protected double servoMotionDetectingMaxAvailableForce = 0.0; // 力保持方向可接受的最大力值
        protected double servoMotionDetectingMinAvailableForce = 0.0; // 力保持方向可接受的最小力值
        protected double servoMotionVibratingPeriodicalMaxIncrement = 0.0; // 摆动方向每周期最大弧度增量

        protected double servoMotionVibratingCurrentAngle = 0.0; // 当前姿态相对改变角度

        protected double servoMotionTargetForceKeep = 0.0; // 目标保持力

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
            get { return (double)ServoMotionModuleFlag.TangentialTranslationWithForce; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoTangentialTranslationWithForce(URDataProcessor Processor, bool Port30004Used)
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
        /// <param name="VibrateForceToAngle">在摆动方向上的力误差对应的角度偏差</param>
        /// <param name="VibrateAngleToForce">在摆动方向上的角度误差对应的力偏差</param>
        /// <param name="TargetForceKeep">目标保持力</param>
        /// <param name="IfRotate">是否改变姿态</param>
        /// <param name="UpAngle">姿态改变角度上限</param>
        /// <param name="IfFindInitialAngle">是否寻找初始姿态角</param>
        /// <param name="IfRecorrectRotateAngle">是否矫正姿态角</param>
        /// <param name="JudgeStep">矫正步长，隔多少距离矫正一次</param>
        /// <param name="IgnoreJudgeSpan">忽略矫正长度，开始的时候隔多少距离开始进入矫正状态</param>
        /// <param name="OverlappingRate">矫正采样重叠范围</param>
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
                                                                    double VibrateForceToAngle,
                                                                    double VibrateAngleToForce,
                                                                    double TargetForceKeep,
                                                                    bool IfRotate,
                                                                    double UpAngle,
                                                                    bool IfFindInitialAngle = false,
                                                                    bool IfRecorrectRotateAngle = false,
                                                                    double IgnoreJudgeSpan = 0.001,
                                                                    double JudgeStep = 0.003,
                                                                    double OverlappingRate = 0.5,
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

            // 设定摆动方向的角度与力之间的转换关系
            servoMotionFromFrictionToAngle = VibrateForceToAngle;
            servoMotionFromAngleToFriction = VibrateAngleToForce;

            // 设定目标保持力
            servoMotionTargetForceKeep = TargetForceKeep;

            // 设定姿态是否更改，以及更改的上下限
            servoMotionIfAttitudeChange = IfRotate;
            servoMotionUpBoundAngle = UpAngle;

            // 设定是否寻找初始姿态角
            servoMotionIfFindInitialAngle = IfFindInitialAngle;

            // 设定是否矫正姿态角
            servoMotionIfRecorrectRotateAngle = IfRecorrectRotateAngle;
            servoMotionIgnoreJudgeSpan = IgnoreJudgeSpan;
            servoMotionJudgeStep = JudgeStep;
            servoMotionOverlappingRate = OverlappingRate;

            // 设置伺服参数并重写下位机控制程序
            SetServoMotionParameters(ControlPeriod, LookAheadTime, Gain);
            internalProcessor.WriteStringToControlCode(ControlPeriod, LookAheadTime, Gain);

            // 准备查找初始姿态角
            RotateAboutInitialAngle();
        }

        /// <summary>
        /// 伺服运动模块旋转一定的初始角度
        /// </summary>
        protected async void RotateAboutInitialAngle()
        {
            if (servoMotionIfFindInitialAngle)
            {
                await Task.Delay(500);

                await Task.Run(new Action(() =>
                {
                    double[] forces = internalProcessor.ServoGetAverageForces();
                    double forceOnMoving = URMath.VectorDotMultiply(servoMotionMovingDirectionArrayAtTcp, new double[] { forces[0], forces[1], forces[2] });
                    double forceOnDetecting = URMath.VectorDotMultiply(servoMotionDetectingDirectionArrayAtTcp, new double[] { forces[0], forces[1], forces[2] });

                    if (forceOnMoving < 0.0)
                    {
                        servoMotionInitialAngle = 0.0;
                    }
                    else
                    {
                        double theta = Math.Atan2(Math.Abs(forceOnMoving), Math.Abs(forceOnDetecting));
                        if (theta < 0.0 || theta > Math.PI / 2.0)
                        {
                            servoMotionInitialAngle = 0.0;
                        }

                        servoMotionInitialAngle = theta;
                    }
                }));

                await Task.Run(new Action(() =>
                {
                    double[] tcpCurrentPosition = internalProcessor.PositionsTcpActual;
                    double[] nextPosition = new double[] { tcpCurrentPosition[0], tcpCurrentPosition[1], tcpCurrentPosition[2] };
                    for (int k = 0; k < 3; k++)
                    {
                        nextPosition[k] += servoMotionInitialDetectingDirectionArrayAtBase[k] * (-servoMotionDetectForceBackDistance);
                    }

                    double[] nextPosture = URMath.Quatnum2AxisAngle(
                                          URMath.QuatnumRotate(new Quatnum[] { 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { tcpCurrentPosition[3], tcpCurrentPosition[4], tcpCurrentPosition[5] }), 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { servoMotionInitialAngle * servoMotionInitialVibratingDirectionArrayAtBase[0], servoMotionInitialAngle * servoMotionInitialVibratingDirectionArrayAtBase[1], servoMotionInitialAngle * servoMotionInitialVibratingDirectionArrayAtBase[2] }) }));

                    internalProcessor.SendURCommanderMoveL(new double[] { nextPosition[0], nextPosition[1], nextPosition[2], nextPosture[0], nextPosture[1], nextPosture[2] }, 0.1, 0.1);
                }));

                await Task.Run(new Action(() =>
                {
                    Thread.Sleep(2000);
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                    }

                    // 执行扫查力度校验，方向根据所测得的力定
                    Thread.Sleep(200);
                    internalProcessor.nonServoFindForceTranslationModule.NonServoMotionSetAndBegin((URNonServo.NonServoFindForceTranslation.NonServoDirectionAtTcp)((byte)servoMotionDetectingDirectionAtTcp),
                                                                                                                                                             internalProcessor.PositionsTcpActual,
                                                                                                                                                             0.0005, 0.005,
                                                                                                                                                             servoMotionTargetForceKeep, true);

                    Thread.Sleep(800);
                    while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                    {
                        Thread.Sleep(200);
                    }
                }));
            }

            // 初始化力保持的值
            servoMotionPreservedForce[0] = 0.0;

            // 初始化姿态实时相对角度
            servoMotionVibratingCurrentAngle = 0.0;

            // 初始化其余必要控制参数
            servoMotionAdjustBias = servoMotionAdjustFrictionBiasIni;
            servoMotionInstantBias = servoMotionAdjustFrictionBiasIni;
            servoMotionMovingDirectionCordinates.Clear();
            servoMotionDetectingForceDirectionCordinates.Clear();
            servoMotionVibratingAngleCordinates.Clear();
            servoMotionVibratingAnglePredict.Clear();
            servoMotionVibratingAnglePredictAfterFilter.Clear();
            servoMotionVibratingAnglePredictAfterAccum.Clear();
            servoMotionLastIndex = 0;
            servoMotionVibrateAngleModifyBegin = false;
            servoMotionVibrateAngleIgnoreRegionBeyond = false;
            servoMotionMeanAngleChange = 0.0;
            servoMotionAccmuErr = 0.0;

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
            double differenceForce = URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp) - servoMotionPreservedForce[0];
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

            for (int k = 0; k < 3; k++)
            {
                nextTcpPosition[k] += detectingDirectionAtBase[k] * forceDirectionIncrement;
            }

            // 当前运动坐标系中的坐标获取
            double[] arrayP = new double[] { tcpRealPosition[0] - servoMotionBeginTcpPosition[0], tcpRealPosition[1] - servoMotionBeginTcpPosition[1], tcpRealPosition[2] - servoMotionBeginTcpPosition[2] };
            double coordinateV = URMath.VectorDotMultiply(arrayP, servoMotionInitialMovingDirectionArrayAtBase);
            double coordinateD = URMath.VectorDotMultiply(arrayP, servoMotionInitialDetectingDirectionArrayAtBase);

            // 当前角度获得
            double currentAngle = Math.Acos(URMath.VectorDotMultiply(detectingDirectionAtBase, servoMotionInitialDetectingDirectionArrayAtBase));
            currentAngle = currentAngle < 0.0 ? 0.0 : currentAngle;
            currentAngle = currentAngle > Math.PI / 2.0 ? Math.PI / 2.0 : currentAngle;

            // 保存关键数据
            servoMotionMovingDirectionCordinates.Add(coordinateV);
            servoMotionDetectingForceDirectionCordinates.Add(coordinateD);
            servoMotionVibratingAngleCordinates.Add(currentAngle);
            int currentIndex = servoMotionVibratingAngleCordinates.Count - 1;
            double[] distanceJudgeArray = new double[] { coordinateV - servoMotionMovingDirectionCordinates[servoMotionLastIndex], coordinateD - servoMotionDetectingForceDirectionCordinates[servoMotionLastIndex], 0.0 };

            // 矫正摩擦力控制 离散式
            double deltaForce = 0.0;
            double judgeThisRound = 0.0;
            double theta = 0.0;
            if (!servoMotionVibrateAngleIgnoreRegionBeyond && URMath.LengthOfArray(distanceJudgeArray) > servoMotionIgnoreJudgeSpan)
            {
                servoMotionVibrateAngleIgnoreRegionBeyond = true;
                servoMotionLastIndex = currentIndex;
            }
            if (!servoMotionVibrateAngleModifyBegin && URMath.LengthOfArray(distanceJudgeArray) > servoMotionJudgeStep)
            {
                servoMotionVibrateAngleModifyBegin = true;
            }
            if (servoMotionIfRecorrectRotateAngle && servoMotionVibrateAngleModifyBegin)
            {
                if (URMath.LengthOfArray(distanceJudgeArray) > servoMotionJudgeStep)
                {
                    theta = GetPredictAngleAfterLimitation(servoMotionLastIndex, currentIndex);

                    deltaForce = TuneFrictionTranslation(servoMotionVibratingAnglePredictAfterAccum.Count - 1, servoMotionVibratingAnglePredictAfterAccum.Count - 1, servoMotionLastIndex, currentIndex);
                    judgeThisRound = 1.0;

                    servoMotionLastIndex = (int)Math.Round(currentIndex - (currentIndex - servoMotionLastIndex) * servoMotionOverlappingRate);
                }
            }

            // 当前正压力和摩擦力
            double currentFraction = -URMath.VectorDotMultiply(referenceForce, servoMotionMovingDirectionArrayAtTcp);
            double currentNormForce = -URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionArrayAtTcp);

            // 摩擦力控制摆动
            double factor = 1.0 / (1.0 + Math.Exp(-8.0 * currentNormForce + 20.0));
            double refFraction = (1 - factor) * (0.07632 * currentNormForce + 0.02814) + factor * (0.1404 * currentNormForce + 0.02702) + servoMotionAdjustBias;
            double diffFraction = refFraction - currentFraction;
            double deltaAngle = diffFraction * servoMotionFromFrictionToAngle;

            // 摆动速度限幅
            if (Math.Abs(deltaAngle) > servoMotionVibratingPeriodicalMaxIncrement)
            {
                deltaAngle = Math.Sign(deltaAngle) * servoMotionVibratingPeriodicalMaxIncrement;
            }

            double predictAngle = currentAngle + deltaAngle - servoMotionInitialAngle;
            if (predictAngle < 0) predictAngle = 0;
            if (predictAngle > servoMotionUpBoundAngle - servoMotionInitialAngle) predictAngle = servoMotionUpBoundAngle - servoMotionInitialAngle;

            double[] nextPosture = URMath.Quatnum2AxisAngle(
                                                  URMath.QuatnumRotate(new Quatnum[] { 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] }), 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[0], predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[1], predictAngle * servoMotionInitialVibratingDirectionArrayAtBase[2] }) }));

            if (servoMotionIfAttitudeChange && servoMotionVibrateAngleIgnoreRegionBeyond)
            {
                nextTcpPosition[3] = nextPosture[0];
                nextTcpPosition[4] = nextPosture[1];
                nextTcpPosition[5] = nextPosture[2];
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
                                                                                    coordinateV, 
                                                                                    coordinateD, 
                                                                                    currentAngle,
                                                                                    judgeThisRound,
                                                                                    predictAngle,
                                                                                    deltaForce,
                                                                                    servoMotionAdjustBias, 
                                                                                    servoMotionInstantBias,
                                                                                    servoMotionLastIndex, 
                                                                                    theta,
                                                                                    differenceForce, 
                                                                                    forceDirectionIncrement });

            return nextTcpPosition;
        }

        /// <summary>
        /// PCA主方向预测角度
        /// </summary>
        /// <param name="startIndex">起始索引</param>
        /// <param name="endIndex">终止索引</param>
        /// <returns>返回预测角度</returns>
        protected double GetPredictAngleFromPCAMainDirection(int startIndex, int endIndex)
        {
            double xSquareSum = 0.0, ySquareSum = 0.0, xCrossYSum = 0.0;
            double xSum = 0.0, ySum = 0.0;
            for (int k = startIndex; k <= endIndex; ++k)
            {
                xSum += servoMotionMovingDirectionCordinates[k];
                ySum += servoMotionDetectingForceDirectionCordinates[k];
                xSquareSum += servoMotionMovingDirectionCordinates[k] * servoMotionMovingDirectionCordinates[k];
                ySquareSum += servoMotionDetectingForceDirectionCordinates[k] * servoMotionDetectingForceDirectionCordinates[k];
                xCrossYSum += servoMotionMovingDirectionCordinates[k] * servoMotionDetectingForceDirectionCordinates[k];
            }

            double U = xSquareSum - xSum * xSum / (endIndex - startIndex + 1);
            double V = ySquareSum - ySum * ySum / (endIndex - startIndex + 1);
            double W = xCrossYSum - xSum * ySum / (endIndex - startIndex + 1);

            return Math.Atan(((V - U) + Math.Sqrt((V - U) * (V - U) + 4 * W * W)) / 2.0 / W);
        }

        /// <summary>
        /// 限制预测的角度
        /// </summary>
        /// <param name="rawAngle">原始预测角度</param>
        /// <returns>返回受限角度</returns>
        protected double LimitPredictAngle(double rawAngle)
        {
            double predictAngle = 0.0;
            if (servoMotionVibratingAnglePredict.Count < 1)
            {
                if (rawAngle < 0.0) predictAngle = 0.0;
                else predictAngle = rawAngle;
            }
            else
            {
                if (rawAngle < 0.0) predictAngle = servoMotionVibratingAnglePredict[servoMotionVibratingAnglePredict.Count - 1];
                else
                {
                    if (servoMotionVibratingAnglePredict.Count > 100)
                    {
                        if (Math.Abs(rawAngle - servoMotionVibratingAnglePredict[servoMotionVibratingAnglePredict.Count - 1]) > 10.0 * Math.Abs(servoMotionMeanAngleChange / (servoMotionVibratingAnglePredict.Count - 1)))
                        {
                            predictAngle = servoMotionVibratingAnglePredict[servoMotionVibratingAnglePredict.Count - 1];
                        }
                        else predictAngle = rawAngle;
                    }
                    else predictAngle = rawAngle;
                }
                servoMotionMeanAngleChange += (predictAngle - servoMotionVibratingAnglePredict[servoMotionVibratingAnglePredict.Count - 1]);
            }

            servoMotionVibratingAnglePredict.Add(predictAngle);

            // 角度滤波
            if (servoMotionVibratingAnglePredictAfterFilter.Count < 1)
            {
                servoMotionVibratingAnglePredictAfterFilter.Add(predictAngle);

                servoMotionVibratingAnglePredictAfterAccum.Add(predictAngle);

                return predictAngle;
            }
            else
            {
                int endIndex = servoMotionVibratingAnglePredict.Count - 1;
                double limitation = servoMotionJudgeStep * servoMotionOverlappingRate / 0.001 * servoMotionMaxAngleDiff;
                if (servoMotionVibratingAnglePredict[endIndex] - servoMotionVibratingAnglePredict[endIndex - 1] > limitation)
                {
                    predictAngle = servoMotionAlpha * (servoMotionVibratingAnglePredict[endIndex - 1] + limitation) + (1 - servoMotionAlpha) * servoMotionVibratingAnglePredictAfterFilter[endIndex - 1];
                }
                else if (servoMotionVibratingAnglePredict[endIndex] - servoMotionVibratingAnglePredict[endIndex - 1] < -limitation)
                {
                    predictAngle = servoMotionAlpha * (servoMotionVibratingAnglePredict[endIndex - 1] - limitation) + (1 - servoMotionAlpha) * servoMotionVibratingAnglePredictAfterFilter[endIndex - 1];
                }
                else
                {
                    predictAngle = servoMotionAlpha * servoMotionVibratingAnglePredict[endIndex - 1] + (1 - servoMotionAlpha) * servoMotionVibratingAnglePredictAfterFilter[endIndex - 1];
                }

                servoMotionVibratingAnglePredictAfterFilter.Add(predictAngle);

                double tempErr = servoMotionAccmuErr * endIndex;
                tempErr += (servoMotionVibratingAnglePredict[endIndex - 1] - predictAngle);
                servoMotionAccmuErr = tempErr / (endIndex + 1);

                servoMotionVibratingAnglePredictAfterAccum.Add(predictAngle + servoMotionAccmuErr);

                return predictAngle + servoMotionAccmuErr;
            }
        }

        /// <summary>
        /// 获得受限预测角度
        /// </summary>
        /// <param name="startIndex">起始索引</param>
        /// <param name="endIndex">终止索引</param>
        /// <returns>返回受限预测角度</returns>
        protected double GetPredictAngleAfterLimitation(int startIndex, int endIndex)
        {
            return LimitPredictAngle(GetPredictAngleFromPCAMainDirection(startIndex, endIndex));
        }

        /// <summary>
        /// 调整摩擦力参数
        /// </summary>
        /// <param name="fakeL">预测角序列下限</param>
        /// <param name="fakeU">预测角序列上限</param>
        /// <param name="realL">实测角序列下限</param>
        /// <param name="realU">实测角序列上限</param>
        protected double TuneFrictionTranslation(int fakeL, int fakeU, int realL, int realU)
        {
            // 比较均值
            double predictAvgAngle = 0.0, realAvgAngle = 0.0;

            for (int k = fakeL; k <= fakeU; ++k)
            {
                predictAvgAngle += servoMotionVibratingAnglePredictAfterAccum[k];
            }
            predictAvgAngle /= (fakeU - fakeL + 1);

            for (int k = realL; k <= realU; ++k)
            {
                realAvgAngle += servoMotionVibratingAngleCordinates[k];
            }
            realAvgAngle /= (realU - realL + 1);

            realAvgAngle = servoMotionVibratingAngleCordinates[realU];

            // 所用角度差
            double deltaAngle = servoMotionModifyAngleTrustCoeff * (predictAvgAngle - realAvgAngle);

            double deltaForce = deltaAngle * servoMotionFromAngleToFriction;

            if (Math.Abs(deltaForce) > servoMotionMaxFrictionForceBias)
            {
                deltaForce = Math.Sign(deltaForce) * servoMotionMaxFrictionForceBias;
            }

            servoMotionInstantBias = servoMotionAdjustFrictionBiasIni + deltaForce;

            servoMotionAdjustBias = (1 - servoMotionModifyAngleTrustCoeff) * servoMotionAdjustBias + servoMotionModifyAngleTrustCoeff * servoMotionInstantBias;

            return deltaForce;
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
                                              servoMotionFromFrictionToAngle,
                                              servoMotionIfFindInitialAngle ? 1.0 : 0.0,
                                              servoMotionInitialAngle,
                                              servoMotionIfRecorrectRotateAngle ? 1.0 : 0.0,
                                              servoMotionIgnoreJudgeSpan,
                                              servoMotionJudgeStep,
                                              servoMotionOverlappingRate,
                                              servoMotionFromAngleToFriction,
                                              servoMotionIfAttitudeChange ? 1.0 : 0.0,
                                              servoMotionUpBoundAngle };
        }














































        #endregion
    }
}
