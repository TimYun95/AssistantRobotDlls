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
    /// 伺服球面运动模块
    /// </summary>
    public class ServoSphereTranslation : ServoMotionBase
    {
        #region 枚举
        /// <summary>
        /// 某轴所使用的力信号维度
        /// </summary>
        protected enum ServoOccupiedForce : byte
        {
            ForceX = 0,
            ForceY,
            ForceZ
        }
        #endregion

        #region 字段
        protected double[] servoMotionPointedPosition = new double[3]; // 指向的目标位置

        protected double servoMotionUpBiasAngle = 0.0; // 运动上偏角
        protected double servoMotionDownBiasAngle = 0.0; // 运动下偏角
        protected double servoMotionLeftBiasAngle = 0.0; // 运动左偏角
        protected double servoMotionRightBiasAngle = 0.0; // 运动右偏角

        protected double servoMotionKeepDistance = 0.0; // 保持到目标位置的距离
        protected ServoOccupiedForce servoMotionLongitudeForce = ServoOccupiedForce.ForceX; // 经度方向力信号
        protected ServoOccupiedForce servoMotionLatitudeForce = ServoOccupiedForce.ForceY; // 纬度方向力信号
        protected int servoMotionLongtitudeForceSign = 1; // 经度方向力信号符号
        protected int servoMotionLatitudeForceSign = 1; // 纬度方向力信号符号

        protected double servoMotionLongitudeRefAngle = 0.0; // 经度基准角
        protected double[] servoMotionLatitudeRefAngleArray = new double[3]; // 纬度基准矢量 
        protected double[] servoMotionLatitudeTangentialArray = new double[3]; // 纬度切向方向

        protected double servoMotionMaxIncrement = 0.0; // 每周期最大位移增量
        protected double servoMotionAngleMaxIncrement = 0.0; // 每周期最大角位移增量
        protected double servoMotionMaxAvailableForce = 0.0; // 单轴可接受的最大力值
        protected double servoMotionMinAvailableForce = 0.0; // 单轴可接受的最小力值

        protected double[] servoMotionLastNextPositionOnSphere = new double[6]; // 前次球面上的目标点位置

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
            get { return (double)ServoMotionModuleFlag.SphereTranslation; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoSphereTranslation(URDataProcessor Processor, bool Port30004Used)
            : base(Processor, Port30004Used) { }

        /// <summary>
        /// 伺服运动模式设置并开始
        /// </summary>
        /// <param name="PointedPosition">指向的肿瘤位置</param>
        /// <param name="MaxSpeed">单轴移动最大速度</param>
        /// <param name="MaxForce">单轴允许最大输入力</param>
        /// <param name="MinForce">单轴允许最小输入力</param>
        /// <param name="UpLimitAngle">向上偏转最大角度</param>
        /// <param name="DownLimitAngle">向下偏转最大角度</param>
        /// <param name="LeftLimitAngle">向左偏转最大角度</param>
        /// <param name="RightLimitAngle">向右偏转最大角度</param>    
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(double[] PointedPosition,
                                                                    double MaxSpeed,
                                                                    double MaxForce,
                                                                    double MinForce,
                                                                    double UpLimitAngle,
                                                                    double DownLimitAngle,
                                                                    double LeftLimitAngle,
                                                                    double RightLimitAngle,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设置目标位置和初始位置
            servoMotionPointedPosition = (double[])PointedPosition.Clone();

            // 设置移动的最大速度，以及相应的力值
            servoMotionMaxIncrement = MaxSpeed;
            servoMotionMaxAvailableForce = MaxForce;
            servoMotionMinAvailableForce = MinForce;

            // 设置能够移动的偏置角度
            servoMotionUpBiasAngle = UpLimitAngle / 180.0 * Math.PI;
            servoMotionDownBiasAngle = DownLimitAngle / 180.0 * Math.PI;
            servoMotionLeftBiasAngle = LeftLimitAngle / 180.0 * Math.PI;
            servoMotionRightBiasAngle = RightLimitAngle / 180.0 * Math.PI;

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
                case 0: // 第一个周期 传参端口设置
                    if (ifPort30004Used)
                    {
                        internalProcessor.SendURServorInputSetup();
                    }
                    break;
                case 1:
                    break;
                case 2: // 第三个周期 清空数据记录 获得Tcp位置，记录并下发传参端口作为初始值
                    servoMotionRecordDatas.Clear();

                    servoMotionBeginTcpPosition = (double[])tcpRealPosition.Clone();
                    servoMotionLastNextPositionOnSphere[0] = servoMotionBeginTcpPosition[0];
                    servoMotionLastNextPositionOnSphere[1] = servoMotionBeginTcpPosition[1];
                    servoMotionLastNextPositionOnSphere[2] = servoMotionBeginTcpPosition[2];
                    servoMotionLastNextPositionOnSphere[3] = servoMotionBeginTcpPosition[3];
                    servoMotionLastNextPositionOnSphere[4] = servoMotionBeginTcpPosition[4];
                    servoMotionLastNextPositionOnSphere[5] = servoMotionBeginTcpPosition[5];

                    if (ifPort30004Used)
                    {
                        internalProcessor.SendURServorInputDatas(servoMotionBeginTcpPosition);
                    }
                    else
                    {
                        internalProcessor.SendURModbusInputDatas(servoMotionBeginTcpPosition);
                    }
                    break;
                case 3: // 第四个周期 计算基准角度或向量                    
                    servoMotionToolDirectionXAtBase = internalProcessor.XDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                    servoMotionToolDirectionYAtBase = internalProcessor.YDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                    servoMotionToolDirectionZAtBase = internalProcessor.ZDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);

                    double[] initialArray = new double[] { servoMotionBeginTcpPosition[0] - servoMotionPointedPosition[0], servoMotionBeginTcpPosition[1] - servoMotionPointedPosition[1], servoMotionBeginTcpPosition[2] - servoMotionPointedPosition[2] };
                    servoMotionKeepDistance = URMath.LengthOfArray(initialArray);
                    servoMotionAngleMaxIncrement = servoMotionMaxIncrement / servoMotionKeepDistance;
                    double[] initialDirection = new double[] { initialArray[0] / servoMotionKeepDistance, initialArray[1] / servoMotionKeepDistance, initialArray[2] / servoMotionKeepDistance };
                    servoMotionLongitudeRefAngle = (Math.Abs(initialDirection[2]) > 1.0) ? Math.Acos((double)Math.Sign(initialDirection[2])) : Math.Acos(initialDirection[2]);
                    double servoMotionLatitudeRefAngleArrayMagnitude = URMath.LengthOfArray(new double[] { initialDirection[0], initialDirection[1], 0.0 });
                    servoMotionLatitudeRefAngleArray[0] = initialDirection[0] / servoMotionLatitudeRefAngleArrayMagnitude;
                    servoMotionLatitudeRefAngleArray[1] = initialDirection[1] / servoMotionLatitudeRefAngleArrayMagnitude;
                    servoMotionLatitudeRefAngleArray[2] = 0.0;

                    byte rollMode = internalProcessor.RollFlag;
                    switch (rollMode)
                    {
                        case 0:
                            servoMotionLongitudeForce = ServoOccupiedForce.ForceX;
                            servoMotionLatitudeForce = ServoOccupiedForce.ForceY;
                            servoMotionLongtitudeForceSign = 1;
                            servoMotionLatitudeForceSign = 1;
                            servoMotionLatitudeTangentialArray = internalProcessor.YDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                            break;
                        case 1:
                            servoMotionLongitudeForce = ServoOccupiedForce.ForceY;
                            servoMotionLatitudeForce = ServoOccupiedForce.ForceX;
                            servoMotionLongtitudeForceSign = -1;
                            servoMotionLatitudeForceSign = 1;
                            servoMotionLatitudeTangentialArray = internalProcessor.XDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                            break;
                        case 2:
                            servoMotionLongitudeForce = ServoOccupiedForce.ForceX;
                            servoMotionLatitudeForce = ServoOccupiedForce.ForceY;
                            servoMotionLongtitudeForceSign = -1;
                            servoMotionLatitudeForceSign = -1;
                            servoMotionLatitudeTangentialArray = internalProcessor.YDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                            servoMotionLatitudeTangentialArray[0] = -servoMotionLatitudeTangentialArray[0];
                            servoMotionLatitudeTangentialArray[1] = -servoMotionLatitudeTangentialArray[1];
                            servoMotionLatitudeTangentialArray[2] = -servoMotionLatitudeTangentialArray[2];
                            break;
                        case 3:
                            servoMotionLongitudeForce = ServoOccupiedForce.ForceY;
                            servoMotionLatitudeForce = ServoOccupiedForce.ForceX;
                            servoMotionLongtitudeForceSign = 1;
                            servoMotionLatitudeForceSign = -1;
                            servoMotionLatitudeTangentialArray = internalProcessor.XDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                            servoMotionLatitudeTangentialArray[0] = -servoMotionLatitudeTangentialArray[0];
                            servoMotionLatitudeTangentialArray[1] = -servoMotionLatitudeTangentialArray[1];
                            servoMotionLatitudeTangentialArray[2] = -servoMotionLatitudeTangentialArray[2];
                            break;
                        default:
                            servoMotionLongitudeForce = ServoOccupiedForce.ForceX;
                            servoMotionLatitudeForce = ServoOccupiedForce.ForceY;
                            servoMotionLongtitudeForceSign = 1;
                            servoMotionLatitudeForceSign = 1;
                            servoMotionLatitudeTangentialArray = internalProcessor.YDirectionOfTcpAtBaseReference(servoMotionBeginTcpPosition);
                            break;
                    }
                    break;
                case 4: // 第五个周期 下达下位机指令并运行
                    servoMotionPreservedForce[0] = 0.0;
                    servoMotionPreservedForce[1] = 0.0;
                    servoMotionPreservedForce[2] = 0.0;

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

            // 计算当前位置在球面上的最佳投影位置
            double[] realArray = new double[] { tcpRealPosition[0] - servoMotionPointedPosition[0], tcpRealPosition[1] - servoMotionPointedPosition[1], tcpRealPosition[2] - servoMotionPointedPosition[2] };
            double realArrayMagnitude = URMath.LengthOfArray(realArray);
            double[] realDirection = new double[] { realArray[0] / realArrayMagnitude, realArray[1] / realArrayMagnitude, realArray[2] / realArrayMagnitude };
            double[] realLatitudeDirection = new double[] { realDirection[0], realDirection[1], 0.0 };
            double realLongitudeAngle = servoMotionLongitudeRefAngle - ((Math.Abs(realDirection[2]) > 1.0) ? Math.Acos((double)Math.Sign(realDirection[2])) : Math.Acos(realDirection[2]));
            double realLatitudeAngleMagnitudeCos = URMath.VectorDotMultiply(realLatitudeDirection, servoMotionLatitudeRefAngleArray) / URMath.LengthOfArray(realLatitudeDirection) / URMath.LengthOfArray(servoMotionLatitudeRefAngleArray);
            double realLatitudeAngleMagnitude = Math.Abs(realLatitudeAngleMagnitudeCos) > 1.0 ? Math.Acos((double)Math.Sign(realLatitudeAngleMagnitudeCos)) : Math.Acos(realLatitudeAngleMagnitudeCos);
            double[] realCrossResult = URMath.VectorCrossMultiply(realLatitudeDirection, servoMotionLatitudeRefAngleArray);
            double realLatitudeAngle = realCrossResult[2] > 0 ? -realLatitudeAngleMagnitude : realLatitudeAngleMagnitude;

            // 计算力传感器给出的移动角度
            double longitudeForce = 0.0;
            double latitudeForce = 0.0;
            switch (servoMotionLongitudeForce)
            {
                case ServoOccupiedForce.ForceX:
                    longitudeForce = referenceForce[0] * servoMotionLongtitudeForceSign;
                    break;
                case ServoOccupiedForce.ForceY:
                    longitudeForce = referenceForce[1] * servoMotionLongtitudeForceSign;
                    break;
                default:
                    longitudeForce = 0.0;
                    break;
            }
            switch (servoMotionLatitudeForce)
            {
                case ServoOccupiedForce.ForceX:
                    latitudeForce = referenceForce[0] * servoMotionLatitudeForceSign;
                    break;
                case ServoOccupiedForce.ForceY:
                    latitudeForce = referenceForce[1] * servoMotionLatitudeForceSign;
                    break;
                default:
                    latitudeForce = 0.0;
                    break;
            }

            double longitudeAngle = 0.0;
            double latitudeAngle = 0.0;
            if (Math.Abs(longitudeForce) < servoMotionMinAvailableForce) longitudeAngle = 0.0;
            else if (Math.Abs(longitudeForce) > servoMotionMaxAvailableForce) longitudeAngle = Math.Sign(longitudeForce) * servoMotionAngleMaxIncrement;
            else longitudeAngle = Math.Sign(longitudeForce) * (Math.Abs(longitudeForce) - servoMotionMinAvailableForce) / (servoMotionMaxAvailableForce - servoMotionMinAvailableForce) * servoMotionAngleMaxIncrement;
            if (Math.Abs(latitudeForce) < servoMotionMinAvailableForce) latitudeAngle = 0.0;
            else if (Math.Abs(latitudeForce) > servoMotionMaxAvailableForce) latitudeAngle = Math.Sign(latitudeForce) * servoMotionAngleMaxIncrement;
            else latitudeAngle = Math.Sign(latitudeForce) * (Math.Abs(latitudeForce) - servoMotionMinAvailableForce) / (servoMotionMaxAvailableForce - servoMotionMinAvailableForce) * servoMotionAngleMaxIncrement;

            // 根据角度限制目标角度
            double aimLongitudeAngle = realLongitudeAngle + longitudeAngle;
            aimLongitudeAngle = aimLongitudeAngle > servoMotionUpBiasAngle ? servoMotionUpBiasAngle : aimLongitudeAngle;
            aimLongitudeAngle = aimLongitudeAngle < -servoMotionDownBiasAngle ? -servoMotionDownBiasAngle : aimLongitudeAngle;
            double aimLatitudeAngle = realLatitudeAngle + latitudeAngle;
            aimLatitudeAngle = aimLatitudeAngle > servoMotionRightBiasAngle ? servoMotionRightBiasAngle : aimLatitudeAngle;
            aimLatitudeAngle = aimLatitudeAngle < -servoMotionLeftBiasAngle ? -servoMotionLeftBiasAngle : aimLatitudeAngle;

            // 计算目标位置
            double longitudeRotateAngle = (Math.PI / 2.0 - servoMotionLongitudeRefAngle) + aimLongitudeAngle;
            double radiusAfterLongitudeTransfer = servoMotionKeepDistance * Math.Cos(longitudeRotateAngle);
            double[] nextPosition = new double[]  {
                servoMotionPointedPosition[0] + servoMotionLatitudeRefAngleArray[0] * radiusAfterLongitudeTransfer * Math.Cos(aimLatitudeAngle) + servoMotionLatitudeTangentialArray[0] * radiusAfterLongitudeTransfer * Math.Sin(aimLatitudeAngle), 
                servoMotionPointedPosition[1] + servoMotionLatitudeRefAngleArray[1] * radiusAfterLongitudeTransfer * Math.Cos(aimLatitudeAngle) + servoMotionLatitudeTangentialArray[1] * radiusAfterLongitudeTransfer * Math.Sin(aimLatitudeAngle), 
                servoMotionPointedPosition[2] + servoMotionKeepDistance * Math.Sin(longitudeRotateAngle)
            };

            // 计算目标姿态
            Quatnum qFirstRotateFromBaseToTcp = URMath.AxisAngle2Quatnum(new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] });
            Quatnum qSecondRotateLongitudeDirection = URMath.AxisAngle2Quatnum(new double[] { -servoMotionLatitudeTangentialArray[0] * aimLongitudeAngle, -servoMotionLatitudeTangentialArray[1] * aimLongitudeAngle, -servoMotionLatitudeTangentialArray[2] * aimLongitudeAngle });
            Quatnum qThirdRotateLatitudeDirection = URMath.AxisAngle2Quatnum(new double[] { 0.0, 0.0, aimLatitudeAngle });
            double[] nextPosture = URMath.Quatnum2AxisAngle(
                                                 URMath.QuatnumRotate(new Quatnum[] { qFirstRotateFromBaseToTcp, qSecondRotateLongitudeDirection, qThirdRotateLatitudeDirection }));

            // 是否更新目标值
            if (longitudeAngle != 0.0 || latitudeAngle != 0.0)
            {
                servoMotionLastNextPositionOnSphere[0] = nextPosition[0];
                servoMotionLastNextPositionOnSphere[1] = nextPosition[1];
                servoMotionLastNextPositionOnSphere[2] = nextPosition[2];
                servoMotionLastNextPositionOnSphere[3] = nextPosture[0];
                servoMotionLastNextPositionOnSphere[4] = nextPosture[1];
                servoMotionLastNextPositionOnSphere[5] = nextPosture[2];
            }

            // 计算下一个位姿
            nextTcpPosition[0] = servoMotionLastNextPositionOnSphere[0];
            nextTcpPosition[1] = servoMotionLastNextPositionOnSphere[1];
            nextTcpPosition[2] = servoMotionLastNextPositionOnSphere[2];
            nextTcpPosition[3] = servoMotionLastNextPositionOnSphere[3];
            nextTcpPosition[4] = servoMotionLastNextPositionOnSphere[4];
            nextTcpPosition[5] = servoMotionLastNextPositionOnSphere[5];
           
            // 记录数据
            if (servoMotionRecordDatas.Count >= 15000)
            {
                servoMotionRecordDatas.RemoveAt(0);
            }
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
                                              servoMotionPointedPosition[0],
                                              servoMotionPointedPosition[1],
                                              servoMotionPointedPosition[2],
                                              servoMotionKeepDistance,
                                              servoMotionAngleMaxIncrement, 
                                              servoMotionMinAvailableForce, 
                                              servoMotionMaxAvailableForce,
                                              servoMotionUpBiasAngle, 
                                              servoMotionDownBiasAngle, 
                                              servoMotionLeftBiasAngle, 
                                              servoMotionRightBiasAngle };
        }
        #endregion


    }
}

