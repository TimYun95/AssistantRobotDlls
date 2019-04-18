using System;
using System.Collections.Generic;
using System.Reflection;
using URCommunication;
using MathFunction;
using LogPrinter;

namespace URServo
{
    /// <summary>
    /// 伺服直线运动模块
    /// </summary>
    public class ServoStraightTranslation : ServoMotionBase
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
        protected double[] servoMotionMovingDirectionAtBase = new double[3]; // 相对Base坐标系的既定运动方向
        protected double[] servoMotionDetectingDirectionAtBase = new double[3]; // 相对Base坐标系的既定力保持方向
        protected double[] servoMotionVibratingDirectionAtBase = new double[3]; // 相对Base坐标系的既定摆动方向

        protected bool servoMotionIfAttitudeChange = false; // 是否改变姿态
        protected double servoMotionEndAngle = 0.0; // 终点姿态相对改变角度
        protected char servoMotionAngleInterpolationRelyDirection = ' '; // 姿态插值变化的依赖方向

        protected double servoMotionMovingDirectionStopDistance = 0.0; // 在运动方向上的终止距离
        protected double servoMotionDetectingDirectionStopDistance = 0.0; // 在力保持方向上的终止距离
        protected const int servoMotionDetectingDirectionRecurrentCheckStartRound = servoMotionInitialRound + 50;  // 在力保持方向上的回环检查开始轮数
        protected double servoMotionDetectingDirectionRecurrentCheckStopDistance = 0.0; // 在力保持方向上的回环检查的终止距离
        protected int servoMotionDetectingDirectionRecurrentCheckSign = 0; // 在力保持方向上的回环检查的方向符号
        protected ServoStopMode servoMotionActiveDistanceOrRecurrentCondition = ServoStopMode.DistanceCondition; // 采用停止距离条件还是回环停止条件

        protected double servoMotionMovingPeriodicalIncrement = 0.0; // 既定运动每周期增量
        protected double servoMotionDetectingPeriodicalMaxIncrement = 0.0; // 力保持方向每周期最大增量
        protected double servoMotionDetectingPeriodicalMinIncrement = 0.0; // 力保持方向每周期最小增量
        protected double servoMotionDetectingMaxAvailableForce = 0.0; // 力保持方向可接受的最大力值
        protected double servoMotionDetectingMinAvailableForce = 0.0; // 力保持方向可接受的最小力值

        protected double servoMotionProcessAngle = 0.0; // 过程姿态相对改变角度

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
            get { return (double)ServoMotionModuleFlag.StraightTranslation; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoStraightTranslation(URDataProcessor Processor, bool Port30004Used)
            : base(Processor, Port30004Used) { }

        /// <summary>
        /// 伺服运动模块设置并开始
        /// </summary>
        /// <param name="MoveDirectionAtBase">相对Base坐标系的既定运动方向</param>
        /// <param name="DetectDirectionAtBase">相对Base坐标系的既定力保持方向，需注意和运动方向正交</param>
        /// <param name="MoveDirectionStopDistance">在运动方向上的终止距离</param>
        /// <param name="DetectDirectionStopDistance">在力保持方向上的终止距离</param>
        /// <param name="DetectDirectionRecurrentStopDistance">在力保持方向上的回环检查的终止距离</param>
        /// <param name="StopWay">使用的终止条件</param>
        /// <param name="ForwardSpeed">在运动方向上的速度</param>
        /// <param name="ForceMaxSpeed">在力保持方向上的最大速度</param>
        /// <param name="ForceMinSpeed">在力保持方向上的最小速度</param>
        /// <param name="ForceMaxValue">在力保持方向上的最大可接受力值</param>
        /// <param name="ForceMinValue">在力保持方向上的最小可接受力值</param>
        /// <param name="IfRotate">是否改变姿态</param>
        /// <param name="ThatAngle">终点姿态改变角度</param>
        /// <param name="RelyDirection">姿态改变依赖方向，可填m或者d表示运动方向或者力保持方向</param>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(double[] MoveDirectionAtBase,
                                                                    double[] DetectDirectionAtBase,
                                                                    double MoveDirectionStopDistance,
                                                                    double DetectDirectionStopDistance,
                                                                    double DetectDirectionRecurrentStopDistance,
                                                                    ServoStopMode StopWay,
                                                                    double ForwardSpeed,
                                                                    double ForceMaxSpeed,
                                                                    double ForceMinSpeed,
                                                                    double ForceMaxValue,
                                                                    double ForceMinValue,
                                                                    bool IfRotate,
                                                                    double ThatAngle,
                                                                    char RelyDirection,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设定运动和力保持方向
            servoMotionMovingDirectionAtBase = (double[])MoveDirectionAtBase.Clone();
            servoMotionDetectingDirectionAtBase = (double[])DetectDirectionAtBase.Clone();

            // 设定运动和力保持方向的最大行进距离、回环停止距离以及停止条件的选择
            servoMotionMovingDirectionStopDistance = MoveDirectionStopDistance;
            servoMotionDetectingDirectionStopDistance = DetectDirectionStopDistance;
            servoMotionDetectingDirectionRecurrentCheckStopDistance = DetectDirectionRecurrentStopDistance;
            servoMotionActiveDistanceOrRecurrentCondition = StopWay;

            // 设定摆动方向
            servoMotionVibratingDirectionAtBase = URMath.VectorCrossMultiply(servoMotionMovingDirectionAtBase, servoMotionDetectingDirectionAtBase);

            // 设定运动速度
            servoMotionMovingPeriodicalIncrement = ForwardSpeed;

            // 设定力保持方向速度和力限制
            servoMotionDetectingPeriodicalMaxIncrement = ForceMaxSpeed;
            servoMotionDetectingPeriodicalMinIncrement = ForceMinSpeed;
            servoMotionDetectingMaxAvailableForce = ForceMaxValue;
            servoMotionDetectingMinAvailableForce = ForceMinValue;

            // 设定姿态是否更改，以及更改的末值
            servoMotionIfAttitudeChange = IfRotate;
            servoMotionEndAngle = ThatAngle;

            // 设定姿态更改依赖的方向
            servoMotionAngleInterpolationRelyDirection = RelyDirection;

            // 初始化力保持的值
            servoMotionPreservedForce[0] = 0.0;

            // 初始化过程相对角度
            servoMotionProcessAngle = 0.0;

            // 设置伺服参数并重写下位机控制程序
            SetServoMotionParameters(ControlPeriod, LookAheadTime, Gain);
            internalProcessor.WriteStringToControlCode(ControlPeriod, LookAheadTime, Gain);

            // 打开伺服模块时对一般逻辑的处理
            internalProcessor.ServoSwitchMode(true);

            // 开始本伺服模式
            ServoMotionBegin();
        }

        /// <summary>
        /// 伺服运动模块设置并开始
        /// </summary>
        /// <param name="MoveDirectionAtTcp">相对Tcp坐标系的既定运动方向</param>
        /// <param name="DetectDirectionAtTcp">相对Tcp坐标系的既定力保持方向</param>
        /// <param name="tcpCurrentPosition">实时Tcp坐标</param>
        /// <param name="MoveDirectionStopDistance">在运动方向上的终止距离</param>
        /// <param name="DetectDirectionStopDistance">在力保持方向上的终止距离</param>
        /// <param name="DetectDirectionRecurrentStopDistance">在力保持方向上的回环检查的终止距离</param>
        /// <param name="StopWay">使用的终止条件</param>
        /// <param name="ForwardSpeed">在运动方向上的速度</param>
        /// <param name="ForceMaxSpeed">在力保持方向上的最大速度</param>
        /// <param name="ForceMinSpeed">在力保持方向上的最小速度</param>
        /// <param name="ForceMaxValue">在力保持方向上的最大可接受力值</param>
        /// <param name="ForceMinValue">在力保持方向上的最小可接受力值</param>
        /// <param name="IfRotate">是否改变姿态</param>
        /// <param name="ThatAngle">终点姿态改变角度</param>
        /// <param name="RelyDirection">姿态改变依赖方向，可填m或者d表示运动方向或者力保持方向</param>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(ServoDirectionAtTcp MoveDirectionAtTcp,
                                                                    ServoDirectionAtTcp DetectDirectionAtTcp,
                                                                    double[] tcpCurrentPosition,
                                                                    double MoveDirectionStopDistance,
                                                                    double DetectDirectionStopDistance,
                                                                    double DetectDirectionRecurrentStopDistance,
                                                                    ServoStopMode StopWay,
                                                                    double ForwardSpeed,
                                                                    double ForceMaxSpeed,
                                                                    double ForceMinSpeed,
                                                                    double ForceMaxValue,
                                                                    double ForceMinValue,
                                                                    bool IfRotate,
                                                                    double ThatAngle,
                                                                    char RelyDirection,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设定运动和力保持方向
            servoMotionMovingDirectionAtTcp = MoveDirectionAtTcp;
            servoMotionDetectingDirectionAtTcp = DetectDirectionAtTcp;

            // 将运动和力保持方向转换到Base坐标系中
            double[] tcpToBase = URMath.ReverseReferenceRelationship(tcpCurrentPosition);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });
            double[] moveDirectionAtBase = new double[3];
            double[] detectDirectionAtBase = new double[3];
            switch (servoMotionMovingDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeX:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { -1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveY:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeY:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, -1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, 1.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.NegativeZ:
                    moveDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, -1.0 }, qTcpToBase);
                    break;
                default:
                    break;
            }
            switch (servoMotionDetectingDirectionAtTcp)
            {
                case ServoDirectionAtTcp.PositiveX:
                case ServoDirectionAtTcp.NegativeX:
                    detectDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 1.0, 0.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveY:
                case ServoDirectionAtTcp.NegativeY:
                    detectDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 1.0, 0.0 }, qTcpToBase);
                    break;
                case ServoDirectionAtTcp.PositiveZ:
                case ServoDirectionAtTcp.NegativeZ:
                    detectDirectionAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(new double[] { 0.0, 0.0, 1.0 }, qTcpToBase);
                    break;
                default:
                    break;
            }

            // 伺服运动模块设置并开始
            ServoMotionSetAndBegin(moveDirectionAtBase,
                                                      detectDirectionAtBase,
                                                      MoveDirectionStopDistance,
                                                      DetectDirectionStopDistance,
                                                      DetectDirectionRecurrentStopDistance,
                                                      StopWay,
                                                      ForwardSpeed,
                                                      ForceMaxSpeed,
                                                      ForceMinSpeed,
                                                      ForceMaxValue,
                                                      ForceMinValue,
                                                      IfRotate,
                                                      ThatAngle,
                                                      RelyDirection,
                                                      ControlPeriod,
                                                      LookAheadTime,
                                                      Gain);
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
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase);
                    break;
                case 1:
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase);
                    break;
                case 2: // 第三个周期 清空数据记录
                    servoMotionRecordDatas.Clear();
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase);
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
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase);
                    break;
                case 4: // 第五个周期 下达下位机指令并运行
                    servoMotionPreservedForce[0] += URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase);
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

            // 行进方向终止
            double[] moveArray = new double[] { tcpRealPosition[0] - servoMotionBeginTcpPosition[0], tcpRealPosition[1] - servoMotionBeginTcpPosition[1], tcpRealPosition[2] - servoMotionBeginTcpPosition[2] };
            if (URMath.VectorDotMultiply(moveArray, servoMotionMovingDirectionAtBase) > servoMotionMovingDirectionStopDistance)
            {
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Motion direction limitation reached.");
                return true;
            }

            if (servoMotionActiveDistanceOrRecurrentCondition == ServoStopMode.DistanceCondition)
            {
                // 力保持方向距离终止
                if (Math.Abs(URMath.VectorDotMultiply(moveArray, servoMotionDetectingDirectionAtBase)) > servoMotionDetectingDirectionStopDistance)
                {
                    Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Force detection direction limitation reached.");
                    return true;
                }
            }
            else
            {
                // 力保持方向回环终止
                if (servoMotionOpenRound > servoMotionDetectingDirectionRecurrentCheckStartRound)
                {
                    if (servoMotionDetectingDirectionRecurrentCheckSign * URMath.VectorDotMultiply(moveArray, servoMotionDetectingDirectionAtBase) < servoMotionDetectingDirectionRecurrentCheckStopDistance)
                    {
                        Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Force detection direction recurrent reached.");
                        return true;
                    }
                }
                else if (servoMotionOpenRound == servoMotionDetectingDirectionRecurrentCheckStartRound)
                {
                    servoMotionDetectingDirectionRecurrentCheckSign = Math.Sign(URMath.VectorDotMultiply(moveArray, servoMotionDetectingDirectionAtBase));
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
        protected override double[] ServoMotionNextTcpPosition(double[] tcpRealPosition, double[] referenceForce)
        {
            double[] nextTcpPosition = (double[])tcpRealPosition.Clone();

            // 暂停运动
            if (servoMotionActivePause)
            {
                return nextTcpPosition;
            }

            // 运动方向上加一定的增量
            for (int k = 0; k < 3; k++)
            {
                nextTcpPosition[k] += servoMotionMovingDirectionAtBase[k] * servoMotionMovingPeriodicalIncrement;
            }

            // 力保持方向上加一定的增量
            double differenceForce = URMath.VectorDotMultiply(referenceForce, servoMotionDetectingDirectionAtBase) - servoMotionPreservedForce[0];
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
                nextTcpPosition[k] += servoMotionDetectingDirectionAtBase[k] * forceDirectionIncrement;
            }

            double predictAngleBeforeNormalization = 0.0;
            double predictAngle = 0.0;
            // 姿态矫正
            if (servoMotionIfAttitudeChange)
            {
                double[] moveArray = new double[] { tcpRealPosition[0] - servoMotionBeginTcpPosition[0], tcpRealPosition[1] - servoMotionBeginTcpPosition[1], tcpRealPosition[2] - servoMotionBeginTcpPosition[2] };

                double motionProportion = 0.0;
                if (servoMotionAngleInterpolationRelyDirection == 'm')
                {
                    motionProportion = URMath.VectorDotMultiply(moveArray, servoMotionMovingDirectionAtBase) / servoMotionMovingDirectionStopDistance;
                }
                else
                {
                    motionProportion = URMath.VectorDotMultiply(moveArray, servoMotionDetectingDirectionAtBase) / servoMotionDetectingDirectionStopDistance;
                }
                if (motionProportion < 0.0) motionProportion = 0.0;
                if (motionProportion > 1.0) motionProportion = 1.0;

                predictAngleBeforeNormalization = motionProportion * servoMotionEndAngle;
                if (Math.Abs(predictAngleBeforeNormalization) < Math.Abs(servoMotionProcessAngle))
                {
                    predictAngle = servoMotionProcessAngle;
                }
                else
                {
                    predictAngle = predictAngleBeforeNormalization;
                    servoMotionProcessAngle = predictAngle;
                }

                double[] nextPosture = URMath.Quatnum2AxisAngle(
                                                      URMath.QuatnumRotate(new Quatnum[] { 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] }), 
                                                                                                     URMath.AxisAngle2Quatnum(new double[] { predictAngle * servoMotionVibratingDirectionAtBase[0], predictAngle * servoMotionVibratingDirectionAtBase[1], predictAngle * servoMotionVibratingDirectionAtBase[2] }) }));

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
                                                                                    predictAngleBeforeNormalization,
                                                                                    predictAngle });

            return nextTcpPosition;
        }

        /// <summary>
        /// 伺服运动模块部分配置参数输出
        /// </summary>
        /// <returns>配置参数输出</returns>
        protected override double[] ServoMotionOutputConfiguration()
        {
            return new double[]{ ServoMotionFlag,
                                              servoMotionPreservedForce[0],
                                              servoMotionMovingDirectionStopDistance, 
                                              servoMotionDetectingDirectionStopDistance, 
                                              (double)servoMotionActiveDistanceOrRecurrentCondition, 
                                              servoMotionDetectingDirectionRecurrentCheckStopDistance, 
                                              servoMotionMovingPeriodicalIncrement,
                                              servoMotionDetectingPeriodicalMinIncrement,
                                              servoMotionDetectingPeriodicalMaxIncrement,
                                              servoMotionDetectingMinAvailableForce,
                                              servoMotionDetectingMaxAvailableForce,
                                              servoMotionIfAttitudeChange ? 1.0 : 0.0,
                                              servoMotionEndAngle };

        }
        #endregion
    }
}
