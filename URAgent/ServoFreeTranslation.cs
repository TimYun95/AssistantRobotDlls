using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Reflection;

using URCommunication;
using MathFunction;
using LogPrinter;

namespace URServo
{
    /// <summary>
    /// 伺服平移运动模块
    /// </summary>
    public class ServoFreeTranslation : ServoMotionBase
    {
        #region 枚举
        /// <summary>
        /// 允许的运动方向
        /// </summary>
        public enum ServoDirectionAtTcp : byte
        {
            DirectionX = 1,
            DirectionY = 2,
            DirectionZ = 4
        }
        #endregion

        #region 字段
        protected bool servoMotionIfAttitudeVibrate = false; // 是否摆动姿态
        protected double servoMotionVibrateAngle = 0.0; // 摆动姿态相对摆角
        protected ServoDirectionAtTcp servoMotionAttitudeVibrateDirection = ServoDirectionAtTcp.DirectionX; // 姿态摆动的方向

        protected double servoMotionStopDistance = 0.0; // 相对初始位置移动的最远距离

        protected double servoMotionMaxIncrement = 0.0; // 单轴每周期最大增量
        protected double servoMotionMinIncrement = 0.0; // 单轴每周期最小增量
        protected double servoMotionMaxAvailableForce = 0.0; // 单轴可接受的最大力值
        protected double servoMotionMinAvailableForce = 0.0; // 单轴可接受的最小力值
        protected bool servoMotionIfDirectionXEnabled = false; // 伺服移动模块下X方向运动是否被允许
        protected bool servoMotionIfDirectionYEnabled = false; // 伺服移动模块下Y方向运动是否被允许
        protected bool servoMotionIfDirectionZEnabled = false; // 伺服移动模块下Z方向运动是否被允许

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
            get { return (double)ServoMotionModuleFlag.FreeTranslation; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoFreeTranslation(URDataProcessor Processor, bool Port30004Used)
            : base(Processor, Port30004Used) { }

        /// <summary>
        /// 伺服运动模式设置并开始
        /// </summary>
        /// <param name="EnableMoveDirections">允许的移动方向</param>
        /// <param name="MaxSpeed">单轴移动最大速度</param>
        /// <param name="MinSpeed">单轴移动最小速度</param>
        /// <param name="MaxForce">单轴允许最大输入力</param>
        /// <param name="MinForce">单轴允许最小输入力</param>
        /// <param name="StopDistance">移动最远距离</param>
        /// <param name="IfVibrate">是否在移动时摆动姿态</param>
        /// <param name="VibrateAxis">姿态摆动轴</param>
        /// <param name="VibrateAngle">姿态摆动角</param>    
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(ServoDirectionAtTcp EnableMoveDirections,
                                                                    double MaxSpeed,
                                                                    double MinSpeed,
                                                                    double MaxForce,
                                                                    double MinForce,
                                                                    double StopDistance,
                                                                    bool IfVibrate,
                                                                    ServoDirectionAtTcp VibrateAxis,
                                                                    double VibrateAngle,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设置可移动的方向
            servoMotionIfDirectionXEnabled = EnableMoveDirections.HasFlag(ServoDirectionAtTcp.DirectionX);
            servoMotionIfDirectionYEnabled = EnableMoveDirections.HasFlag(ServoDirectionAtTcp.DirectionY);
            servoMotionIfDirectionZEnabled = EnableMoveDirections.HasFlag(ServoDirectionAtTcp.DirectionZ);

            // 设置移动的最大和最小速度，以及对应的力值
            servoMotionMaxIncrement = MaxSpeed;
            servoMotionMinIncrement = MinSpeed;
            servoMotionMaxAvailableForce = MaxForce;
            servoMotionMinAvailableForce = MinForce;

            // 设置能够移动的最远距离
            servoMotionStopDistance = StopDistance;

            // 设置摆动的相关条件
            servoMotionIfAttitudeVibrate = IfVibrate;
            servoMotionAttitudeVibrateDirection = VibrateAxis;
            servoMotionVibrateAngle = VibrateAngle;

            // 初始化力保持的值
            servoMotionPreservedForce[0] = 0.0;
            servoMotionPreservedForce[1] = 0.0;
            servoMotionPreservedForce[2] = 0.0;

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
                    servoMotionPreservedForce[0] += referenceForce[0];
                    servoMotionPreservedForce[1] += referenceForce[1];
                    servoMotionPreservedForce[2] += referenceForce[2];
                    break;
                case 1:
                    servoMotionPreservedForce[0] += referenceForce[0];
                    servoMotionPreservedForce[1] += referenceForce[1];
                    servoMotionPreservedForce[2] += referenceForce[2];
                    break;
                case 2: // 第三个周期 清空数据记录 获得Tcp位置并下发传参端口作为初始值
                    servoMotionRecordDatas.Clear();

                    servoMotionBeginTcpPosition = (double[])tcpRealPosition.Clone();
                    if (ifPort30004Used)
                    {
                        internalProcessor.SendURServorInputDatas(servoMotionBeginTcpPosition);
                    }
                    else
                    {
                        internalProcessor.SendURModbusInputDatas(servoMotionBeginTcpPosition);
                    }

                    servoMotionPreservedForce[0] += referenceForce[0];
                    servoMotionPreservedForce[1] += referenceForce[1];
                    servoMotionPreservedForce[2] += referenceForce[2];
                    break;
                case 3: // 第四个周期 计算各轴在Base坐标系中的坐标
                    servoMotionToolDirectionXAtBase = internalProcessor.XDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                    servoMotionToolDirectionYAtBase = internalProcessor.YDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                    servoMotionToolDirectionZAtBase = internalProcessor.ZDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                    servoMotionPreservedForce[0] += referenceForce[0];
                    servoMotionPreservedForce[1] += referenceForce[1];
                    servoMotionPreservedForce[2] += referenceForce[2];
                    break;
                case 4: // 第五个周期 下达下位机指令并运行
                    servoMotionPreservedForce[0] += referenceForce[0];
                    servoMotionPreservedForce[1] += referenceForce[1];
                    servoMotionPreservedForce[2] += referenceForce[2];
                    servoMotionPreservedForce[0] /= servoMotionInitialRound;
                    servoMotionPreservedForce[1] /= servoMotionInitialRound;
                    servoMotionPreservedForce[2] /= servoMotionInitialRound;

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

            // 可能的运动方向终止
            double[] moveArray = new double[] { tcpRealPosition[0] - servoMotionBeginTcpPosition[0], tcpRealPosition[1] - servoMotionBeginTcpPosition[1], tcpRealPosition[2] - servoMotionBeginTcpPosition[2] };
            if (URMath.LengthOfArray(moveArray) > servoMotionStopDistance)
            {
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Movement edge reached.");
                return true;
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

            // 力保持方向上加一定的增量
            if (servoMotionIfDirectionXEnabled) // X方向上允许移动，则根据力信号误差反馈
            {
                double differenceForceX = referenceForce[0] - servoMotionPreservedForce[0];
                double forceDirectionXIncrement = 0.0;
                if (Math.Abs(differenceForceX) <= servoMotionMinAvailableForce)
                {
                    forceDirectionXIncrement = 0.0;
                }
                else if (Math.Abs(differenceForceX) >= servoMotionMaxAvailableForce)
                {
                    forceDirectionXIncrement = Math.Sign(differenceForceX) * servoMotionMaxIncrement;
                }
                else
                {
                    forceDirectionXIncrement = Math.Sign(differenceForceX) * ((Math.Abs(differenceForceX) - servoMotionMinAvailableForce) / (servoMotionMaxAvailableForce - servoMotionMinAvailableForce) * (servoMotionMaxIncrement - servoMotionMinIncrement) + servoMotionMinIncrement);
                }
                for (int k = 0; k < 3; k++)
                {
                    nextTcpPosition[k] += servoMotionToolDirectionXAtBase[k] * forceDirectionXIncrement;
                }
            }

            if (servoMotionIfDirectionYEnabled) // Y方向上允许移动，则根据力信号误差反馈
            {
                double differenceForceY = referenceForce[1] - servoMotionPreservedForce[1];
                double forceDirectionYIncrement = 0.0;
                if (Math.Abs(differenceForceY) <= servoMotionMinAvailableForce)
                {
                    forceDirectionYIncrement = 0.0;
                }
                else if (Math.Abs(differenceForceY) >= servoMotionMaxAvailableForce)
                {
                    forceDirectionYIncrement = Math.Sign(differenceForceY) * servoMotionMaxIncrement;
                }
                else
                {
                    forceDirectionYIncrement = Math.Sign(differenceForceY) * ((Math.Abs(differenceForceY) - servoMotionMinAvailableForce) / (servoMotionMaxAvailableForce - servoMotionMinAvailableForce) * (servoMotionMaxIncrement - servoMotionMinIncrement) + servoMotionMinIncrement);
                }
                for (int k = 0; k < 3; k++)
                {
                    nextTcpPosition[k] += servoMotionToolDirectionYAtBase[k] * forceDirectionYIncrement;
                }
            }

            if (servoMotionIfDirectionZEnabled) // Z方向上允许移动，则根据力信号误差反馈
            {
                double differenceForceZ = referenceForce[2] - servoMotionPreservedForce[2];
                double forceDirectionZIncrement = 0.0;
                if (Math.Abs(differenceForceZ) <= servoMotionMinAvailableForce)
                {
                    forceDirectionZIncrement = 0.0;
                }
                else if (Math.Abs(differenceForceZ) >= servoMotionMaxAvailableForce)
                {
                    forceDirectionZIncrement = Math.Sign(differenceForceZ) * servoMotionMaxIncrement;
                }
                else
                {
                    forceDirectionZIncrement = Math.Sign(differenceForceZ) * ((Math.Abs(differenceForceZ) - servoMotionMinAvailableForce) / (servoMotionMaxAvailableForce - servoMotionMinAvailableForce) * (servoMotionMaxIncrement - servoMotionMinIncrement) + servoMotionMinIncrement);
                }
                for (int k = 0; k < 3; k++)
                {
                    nextTcpPosition[k] += servoMotionToolDirectionZAtBase[k] * forceDirectionZIncrement;
                }
            }

            // 姿态矫正
            //if (servoLineMotionIfAttitudeChange)
            //{
            //    double[] moveArray = new double[] { positionsTcpActual[0] - servoLineMotionBeginTcpPosition[0], positionsTcpActual[1] - servoLineMotionBeginTcpPosition[1], positionsTcpActual[2] - servoLineMotionBeginTcpPosition[2] };

            //    double motionProportion = 0.0;
            //    if (servoLineMotionAngleInterpolationRelyDirection == 'm')
            //    {
            //        motionProportion = URMath.VectorDotMultiply(moveArray, servoLineMotionSpecificMotionDirectionAtBase) / servoLineMotionSpecificMotionDirectionStopDistance;
            //    }
            //    else
            //    {
            //        motionProportion = URMath.VectorDotMultiply(moveArray, servoLineMotionSpecificForceDetectionDirectionAtBase) / servoLineMotionSpecificForceDirectionStopDistance;
            //    }
            //    if (motionProportion < 0.0) motionProportion = 0.0;
            //    if (motionProportion > 1.0) motionProportion = 1.0;

            //    double planAngle = motionProportion * servoLineMotionEndAngle;
            //    if (Math.Abs(planAngle) < Math.Abs(servoLineMotionProcessAngle))
            //    {
            //        planAngle = servoLineMotionProcessAngle;
            //    }
            //    else
            //    {
            //        servoLineMotionProcessAngle = planAngle;
            //    }

            //    double[] nextPosture = URMath.Quatnum2AxisAngle(
            //                                          URMath.QuatnumRotate(new Quatnum[] { 
            //                                                                                         URMath.AxisAngle2Quatnum(new double[] { servoLineMotionBeginTcpPosition[3], servoLineMotionBeginTcpPosition[4], servoLineMotionBeginTcpPosition[5] }), 
            //                                                                                         URMath.AxisAngle2Quatnum(new double[] { planAngle * servoLineMotionSpecificVibrationDirectionAtBase[0], planAngle * servoLineMotionSpecificVibrationDirectionAtBase[1], planAngle * servoLineMotionSpecificVibrationDirectionAtBase[2] }) }));

            //    nextTcpPosition[3] = nextPosture[0];
            //    nextTcpPosition[4] = nextPosture[1];
            //    nextTcpPosition[5] = nextPosture[2];
            //}

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
                                                                                    referenceForce[5]});

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
                                              servoMotionPreservedForce[1],
                                              servoMotionPreservedForce[2],
                                              (servoMotionIfDirectionXEnabled ? (double)ServoDirectionAtTcp.DirectionX : 0.0) + (servoMotionIfDirectionYEnabled ? (double)ServoDirectionAtTcp.DirectionY : 0.0) + (servoMotionIfDirectionZEnabled ? (double)ServoDirectionAtTcp.DirectionZ : 0.0), 
                                              servoMotionMinIncrement, 
                                              servoMotionMaxIncrement, 
                                              servoMotionMinAvailableForce, 
                                              servoMotionMaxAvailableForce,
                                              servoMotionStopDistance, 
                                              servoMotionIfAttitudeVibrate ? 1.0 : 0.0,
                                              (double)servoMotionAttitudeVibrateDirection,
                                              servoMotionVibrateAngle };
        }
        #endregion


    }
}
