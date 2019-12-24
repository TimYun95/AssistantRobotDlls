using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;

using URCommunication;
using MathFunction;
using LogPrinter;

namespace URServo
{
    /// <summary>
    /// 伺服跟随运动模块
    /// </summary>
    public class ServoTrackTranslation : ServoMotionBase
    {
        #region 字段
        protected double servoMotionStopDistance = 0.0; // 停止距离
        protected double servoMotionMaxDistanceIncrement = 0.0; // 每周期最大位移
        protected double servoMotionMaxAngleIncrement = 0.0; // 每周期最大角移

        protected bool servoMotionForceTrackEnable = true; // 力轴跟随开关
        protected double servoMotionMaxIncrement = 0.0; // 力轴每周期最大增量
        protected double servoMotionMinIncrement = 0.0; // 力轴每周期最小增量
        protected double servoMotionMaxAvailableForce = 0.0; // 力轴可接受的最大力值
        protected double servoMotionMinAvailableForce = 0.0; // 力轴可接受的最小力值

        private static readonly object refLocker = new object(); // 参考值锁
        private static readonly object enableLocker = new object(); // 使能值锁
        protected int servoMotionRefFlag = 0; // 参考标号
        protected double[] servoMotionRefPos = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // 参考位置

        protected List<double[]> servoMotionRecordDatas = new List<double[]>(30000); // 运动过程数据记录（每4分钟一次记录）
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
            get { return (double)ServoMotionModuleFlag.TrackTranslation; }
        }

        /// <summary>
        /// 力轴跟随开关
        /// </summary>
        public bool ServoMotionForceTrackEnable
        {
            set
            {
                lock (enableLocker)
                {
                    servoMotionForceTrackEnable = value;
                }
            }
        }

        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoTrackTranslation(URDataProcessor Processor, bool Port30004Used)
            : base(Processor, Port30004Used) { }

        /// <summary>
        /// 伺服运动模式设置并开始
        /// </summary>
        /// <param name="StopTranslationDistance">伺服运动停止距离</param>
        /// <param name="MaxTranslationLoop">伺服运动单周期最大位移</param>
        /// <param name="MaxRotationLoop">伺服运动单周期最大角移</param>
        /// <param name="IfForceTrackEnable">伺服运动力跟踪使能</param>
        /// <param name="MaxForceSpeedLoop">伺服运动单周期最大力运动距离</param>
        /// <param name="MinForceSpeedLoop">伺服运动单周期最小力运动距离</param>
        /// <param name="MaxForceDiff">伺服运动最大力差</param>
        /// <param name="MinForceDiff">伺服运动最小力差</param>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        public void ServoMotionSetAndBegin(double StopTranslationDistance,
                                                                    double MaxTranslationLoop,
                                                                    double MaxRotationLoop,
                                                                    bool IfForceTrackEnable,
                                                                    double MaxForceSpeedLoop,
                                                                    double MinForceSpeedLoop,
                                                                    double MaxForceDiff,
                                                                    double MinForceDiff,
                                                                    double ControlPeriod = 0.008,
                                                                    double LookAheadTime = 0.1,
                                                                    double Gain = 200)
        {
            // 设置停止和最大距离
            servoMotionStopDistance = StopTranslationDistance;

            // 设置单周期移动最大值
            servoMotionMaxDistanceIncrement = MaxTranslationLoop;
            servoMotionMaxAngleIncrement = MaxRotationLoop;

            // 设置力跟踪使能，力轴移动的最大和最小速度，以及对应的力值
            servoMotionForceTrackEnable = IfForceTrackEnable;
            servoMotionMaxIncrement = MaxForceSpeedLoop;
            servoMotionMinIncrement = MinForceSpeedLoop;
            servoMotionMaxAvailableForce = MaxForceDiff;
            servoMotionMinAvailableForce = MinForceDiff;

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
                    servoMotionRefFlag = 0;
                    servoMotionRefPos[0] = 0.0;
                    servoMotionRefPos[1] = 0.0;
                    servoMotionRefPos[2] = servoMotionBeginTcpPosition[3];
                    servoMotionRefPos[3] = servoMotionBeginTcpPosition[4];
                    servoMotionRefPos[4] = servoMotionBeginTcpPosition[5];

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

                    servoMotionRefPos[5] = servoMotionPreservedForce[2];

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
        /// 伺服运动模块更新参考值
        /// </summary>
        /// <param name="flag">运动标号</param>
        /// <param name="values">运动参考值</param>
        public void ServoMotionRefreshRefValues(int flag, double?[] values)
        {
            lock (refLocker)
            {
                servoMotionRefFlag = flag;
                if (values[0].HasValue) servoMotionRefPos[0] = values[0].Value;
                if (values[1].HasValue) servoMotionRefPos[1] = values[1].Value;
                if (values[2].HasValue) servoMotionRefPos[2] = values[2].Value;
                if (values[3].HasValue) servoMotionRefPos[3] = values[3].Value;
                if (values[4].HasValue) servoMotionRefPos[4] = values[4].Value;
                if (values[5].HasValue) servoMotionRefPos[5] = values[5].Value;
            }
        }

        /// <summary>
        /// 伺服运动模块中计算下一周期的Tcp位置
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        /// <returns>返回下一周期的Tcp位置</returns>
        protected override double[] ServoMotionNextTcpPosition(double[] tcpRealPosition, double[] referenceForce, double[] moreInfo = null)
        {
            int refFlag = 0;
            double[] refDist = new double[] { 0.0, 0.0 };
            double[] refPosture = new double[] { 0.0, 0.0, 0.0 };
            double refForce = 0.0;
            lock (refLocker)
            {
                refFlag = servoMotionRefFlag;
                refDist[0] = servoMotionRefPos[0];
                refDist[1] = servoMotionRefPos[1];
                refPosture[0] = servoMotionRefPos[2];
                refPosture[1] = servoMotionRefPos[3];
                refPosture[2] = servoMotionRefPos[4];
                refForce = servoMotionRefPos[5];
            }

            double[] nextTcpPosition = (double[])tcpRealPosition.Clone();

            // 暂停运动
            if (servoMotionActivePause)
            {
                return nextTcpPosition;
            }

            // 姿态变化，单周期幅值限制
            double[] currentPosture = new double[] { tcpRealPosition[3], tcpRealPosition[4], tcpRealPosition[5] };
            double[] transitPosture = URMath.Quatnum2AxisAngle(
                                                        URMath.FindTransitQuatnum(
                                                            URMath.AxisAngle2Quatnum(currentPosture),
                                                            URMath.AxisAngle2Quatnum(refPosture)));

            double transitAngle = URMath.LengthOfArray(transitPosture);
            double[] transitAxis = new double[] { transitPosture[0] / transitAngle, 
                                                                       transitPosture[1] / transitAngle, 
                                                                       transitPosture[2] / transitAngle };
            if (transitAngle <= servoMotionMaxAngleIncrement)
            {
                nextTcpPosition[3] = refPosture[0];
                nextTcpPosition[4] = refPosture[1];
                nextTcpPosition[5] = refPosture[2];
            }
            else
            {
                double[] newPosture = URMath.Quatnum2AxisAngle(
                    URMath.QuatnumRotate(new Quatnum[] {
                        URMath.AxisAngle2Quatnum(currentPosture),
                        URMath.AxisAngle2Quatnum(new double[] { transitAxis[0] * servoMotionMaxAngleIncrement,
                                                                                           transitAxis[1] * servoMotionMaxAngleIncrement,
                                                                                           transitAxis[2] * servoMotionMaxAngleIncrement })
                }));
                nextTcpPosition[3] = newPosture[0];
                nextTcpPosition[4] = newPosture[1];
                nextTcpPosition[5] = newPosture[2];
            }

            // 位移变化
            double[] currentPosition = new double[] { tcpRealPosition[0], tcpRealPosition[1], tcpRealPosition[2] };
            double[] initialPosition = new double[] { servoMotionBeginTcpPosition[0], servoMotionBeginTcpPosition[1], servoMotionBeginTcpPosition[2] };
            double[] initialPosture = new double[] { servoMotionBeginTcpPosition[3], servoMotionBeginTcpPosition[4], servoMotionBeginTcpPosition[5] };
            double[] aimPointAtPlane = new double[] {
                initialPosition[0] + servoMotionToolDirectionXAtBase[0] * refDist[0] + servoMotionToolDirectionYAtBase[0] * refDist[1], 
                initialPosition[1] + servoMotionToolDirectionXAtBase[1] * refDist[0] + servoMotionToolDirectionYAtBase[1] * refDist[1], 
                initialPosition[2] + servoMotionToolDirectionXAtBase[2] * refDist[0] + servoMotionToolDirectionYAtBase[2] * refDist[1] };
            double[] currentZAxisAtBase = internalProcessor.ZDirectionOfTcpAtBaseReference(tcpRealPosition);

            double enlongedCoeff =
                (currentZAxisAtBase[0] * (currentPosition[0] - aimPointAtPlane[0]) +
                 currentZAxisAtBase[1] * (currentPosition[1] - aimPointAtPlane[1]) +
                 currentZAxisAtBase[2] * (currentPosition[2] - aimPointAtPlane[2])) /
                (currentZAxisAtBase[0] * servoMotionToolDirectionZAtBase[0] +
                 currentZAxisAtBase[1] * servoMotionToolDirectionZAtBase[1] +
                 currentZAxisAtBase[2] * servoMotionToolDirectionZAtBase[2]);
            double[] aimPointAtSurf = new double[] {
                aimPointAtPlane[0] + enlongedCoeff * servoMotionToolDirectionZAtBase[0], 
                aimPointAtPlane[1] + enlongedCoeff * servoMotionToolDirectionZAtBase[1], 
                aimPointAtPlane[2] + enlongedCoeff * servoMotionToolDirectionZAtBase[2] };

            double[] refDelta = new double[] { 
                aimPointAtSurf[0] - currentPosition[0], 
                aimPointAtSurf[1] - currentPosition[1],
                aimPointAtSurf[2] - currentPosition[2]};
            double refDeltaDistance = URMath.LengthOfArray(refDelta);
            double[] refDeltaDirection = new double[] {
                refDelta[0] / refDeltaDistance,
                refDelta[1] / refDeltaDistance,
                refDelta[2] / refDeltaDistance };
            if (refDeltaDistance > servoMotionMaxDistanceIncrement)
            {
                aimPointAtSurf[0] = currentPosition[0] + servoMotionMaxDistanceIncrement * refDeltaDirection[0];
                aimPointAtSurf[1] = currentPosition[1] + servoMotionMaxDistanceIncrement * refDeltaDirection[1];
                aimPointAtSurf[2] = currentPosition[2] + servoMotionMaxDistanceIncrement * refDeltaDirection[2];
            }

            nextTcpPosition[0] = aimPointAtSurf[0];
            nextTcpPosition[1] = aimPointAtSurf[1];
            nextTcpPosition[2] = aimPointAtSurf[2];

            // 力变化
            double forceDirectionZIncrement = 0.0;
            bool ifContinueForceTrack;
            lock (enableLocker)
            {
                ifContinueForceTrack = servoMotionForceTrackEnable;
            }
            if (ifContinueForceTrack)
            {
                double differenceForceZ = referenceForce[2] - (-refForce);
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

                nextTcpPosition[0] += (currentZAxisAtBase[0] * forceDirectionZIncrement);
                nextTcpPosition[1] += (currentZAxisAtBase[1] * forceDirectionZIncrement);
                nextTcpPosition[2] += (currentZAxisAtBase[2] * forceDirectionZIncrement);
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
                                                                                    DateTime.Now.Year,
                                                                                    DateTime.Now.Month,
                                                                                    DateTime.Now.Day,
                                                                                    DateTime.Now.Hour,
                                                                                    DateTime.Now.Minute,
                                                                                    DateTime.Now.Second,
                                                                                    DateTime.Now.Millisecond,
                                                                                    DateTime.Now.Hour * 3600 * 1000 + 
                                                                                    DateTime.Now.Minute * 60 * 1000 + 
                                                                                    DateTime.Now.Second * 1000 + 
                                                                                    DateTime.Now.Millisecond,
                                                                                    refFlag});

            return nextTcpPosition;
        }

        /// <summary>
        /// 伺服运动模块部分配置参数输出
        /// </summary>
        /// <returns>配置参数输出</returns>
        protected override double[] ServoMotionOutputConfiguration()
        {
            bool ifContinueForceTrack;
            lock (enableLocker)
            {
                ifContinueForceTrack = servoMotionForceTrackEnable;
            }
            return new double[]{ ServoMotionFlag,
                                              servoMotionPreservedForce[0],
                                              servoMotionPreservedForce[1],
                                              servoMotionPreservedForce[2],
                                              servoMotionStopDistance,
                                              servoMotionMaxDistanceIncrement,
                                              servoMotionMaxAngleIncrement,
                                              ifContinueForceTrack ? 1.0 : 0.0,
                                              servoMotionMinIncrement, 
                                              servoMotionMaxIncrement, 
                                              servoMotionMinAvailableForce, 
                                              servoMotionMaxAvailableForce };
        }
        #endregion

    }
}
