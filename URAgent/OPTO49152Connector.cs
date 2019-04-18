using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net;
using System.Reflection;
using LogPrinter;

namespace URCommunication
{
    /// <summary>
    /// OPTO中的49152端口通讯类
    /// </summary>
    public class OPTO49152Connector : OPTOUDPBase
    {
        #region 枚举
        /// <summary>
        /// 力参数协议格式首位
        /// </summary>
        protected enum ForceDatasMarks : int
        {
            SequenceNumber = 0,
            SampleCounter = 4,
            SensorStatus = 8,
            ForceX = 12,
            ForceY = 16,
            ForceZ = 20,
            TorqueX = 24,
            TorqueY = 28,
            TorqueZ = 32
        }

        /// <summary>
        /// 传感器数据状态
        /// </summary>
        public enum SensorStatus : int
        {
            Correct = 0,
            Error
        }

        /// <summary>
        /// 传感器种类       
        /// </summary>
        public enum SensorType : byte
        {
            OldOptoForce = 0,
            NewOptoForce,
            OnRobot
        }
        #endregion

        #region 字段 49152端口参数
        protected double forceX = 0.0;
        protected double forceY = 0.0;
        protected double forceZ = 0.0;
        protected double torqueX = 0.0;
        protected double torqueY = 0.0;
        protected double torqueZ = 0.0;
        #endregion

        #region 字段
        private static readonly object lockedObject = new object(); // 线程锁变量
        protected SensorType sensorVersion = SensorType.OldOptoForce; // 传感器种类

        protected CancellationTokenSource listenCancelSource; // 停止监听源
        protected Task listenFromOPTOTask; // OPTO49152端口监听任务

        protected int sampleFrequency = 0; // OPTO采样频率
        protected int frequencyFilter = 0; // OPTO截断频率
        protected int sampleCount = 0; // 要采集的数据个数

        protected const int recievedByteLength = 36; // 接收到的字节长度
        protected int continuousNumOfInvalidRecievedDatas = 0; // 连续收到错误的状态字节数
        protected double[] zeroRestForce = new double[6]; // 力传感器清零后的残余信号
        #endregion

        #region 属性
        /// <summary>
        /// OPTO采样频率
        /// </summary>
        public int SampleFrequency
        {
            get { return sampleFrequency; }
            set { sampleFrequency = value; }
        }

        /// <summary>
        /// OPTO截断频率
        /// </summary>
        public int FrequencyFilter
        {
            get { return frequencyFilter; }
            set { frequencyFilter = value; }
        }

        /// <summary>
        /// 要采集的数据个数
        /// </summary>
        public int SampleCount
        {
            get { return sampleCount; }
            set { sampleCount = value; }
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
        public OPTO49152Connector(SensorType SenorVersion, int FrequencySampling = 250, int FilterSampling = 4, int AmountSampling = 0)
        {
            sensorVersion = SenorVersion;
            sampleFrequency = FrequencySampling;
            frequencyFilter = FilterSampling;
            sampleCount = AmountSampling;

            listenCancelSource = new CancellationTokenSource();
            listenFromOPTOTask = new Task(() => ListenFromOPTOFunction(listenCancelSource.Token));
        }

        /// <summary>
        /// 创建49152端口通讯并连接，必须重载
        /// </summary>
        /// <param name="SelfIP">自身IP地址</param>
        /// <param name="OppositeIP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="Port">公共端口号，默认49152</param>
        public virtual void Creat49152Client(string SelfIP, string OppositeIP, int TimeOut, int Port = 49152)
        {
            CreatClient(SelfIP, OppositeIP, Port, TimeOut);
        }

        /// <summary>
        /// 创建监听49152端口的新任务，并开始监听
        /// </summary>
        public void CreatListenFromOPTOTask()
        {
            if (listenFromOPTOTask.Status.Equals(TaskStatus.Created))
            {
                DeployForceCollector();
                listenFromOPTOTask.Start();
                return;
            }
            else if (listenFromOPTOTask.IsCompleted)
            {
                listenCancelSource = new CancellationTokenSource();
                listenFromOPTOTask = new Task(() => ListenFromOPTOFunction(listenCancelSource.Token));

                DeployForceCollector();
                listenFromOPTOTask.Start();
                return;
            }

            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Try to create OPTO forcedata listener when another listener has not been released yet, so do nothing now.");
        }

        /// <summary>
        /// 配置力信号采集器
        /// </summary>
        protected virtual void DeployForceCollector()
        {
            SendFrequencyCommand(sampleFrequency);
            SendFilterCommand(frequencyFilter);
            SendStartCommand(sampleCount);
        }

        /// <summary>
        /// 停止监听49152端口，阻塞到线程结束
        /// </summary>
        public void StopListenFromOPTOThread()
        {
            if (listenFromOPTOTask.Status.Equals(TaskStatus.Running))
            {
                listenCancelSource.Cancel();

                listenFromOPTOTask.Wait();
                return;
            }

            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Try to stop OPTO forcedata listener when the listener is not running now, so do nothing.");
        }

        /// <summary>
        /// 停止力信号采集器发送数据
        /// </summary>
        protected virtual void EndCollectorSend()
        {
            SendStopCommand();
        }

        /// <summary>
        /// 49152端口监听执行的操作
        /// </summary>
        protected void ListenFromOPTOFunction(CancellationToken CancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "A forcedata listener is going to run.");

            while (true)
            {
                if (CancelFlag.IsCancellationRequested)
                {
                    break;
                }

                UnpackRecievedDatas();
            }

            OPTOConnectionBroken();
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "A forcedata listener is going to stop.");
        }

        /// <summary>
        /// 接收实时力信号并解析，必须重载
        /// </summary>
        protected virtual void UnpackRecievedDatas()
        {
            byte[] getDatas = Response(recievedByteLength);

            if (getDatas.Length < 1) // 收到的字节长度错误
            {
                continuousNumOfInvalidRecievedDatas++;
                if (continuousNumOfInvalidRecievedDatas > 5)
                {
                    continuousNumOfInvalidRecievedDatas = 0;
                    throw new Exception("invalid recieved datas.");
                }
                return;
            }
            else
            {
                int senorStatus = BitConverter.ToInt32(
                                                 BitConverter.GetBytes(
                                                 IPAddress.NetworkToHostOrder(
                                                 BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.SensorStatus))), 0);

                if (senorStatus != (int)SensorStatus.Correct) // 收到的字节顺序错误或者数据错误
                {
                    continuousNumOfInvalidRecievedDatas++;
                    if (continuousNumOfInvalidRecievedDatas > 5)
                    {
                        continuousNumOfInvalidRecievedDatas = 0;
                        throw new Exception("invalid recieved datas.");
                    }
                    return;
                }
                else // 收到的字节正常
                {
                    continuousNumOfInvalidRecievedDatas = 0;
                }
            }

            // 解析收到的字节流
            lock (lockedObject)
            {
                switch (sensorVersion)
                {
                    case SensorType.OldOptoForce:
                        forceY = -(double)IPAddress.NetworkToHostOrder(
                                                     BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceX)) / 10000;
                        forceX = (double)IPAddress.NetworkToHostOrder(
                                                    BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceY)) / 10000;
                        forceZ = (double)IPAddress.NetworkToHostOrder(
                                                    BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceZ)) / 10000;
                        torqueY = -(double)IPAddress.NetworkToHostOrder(
                                                        BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueX)) / 100000;
                        torqueX = (double)IPAddress.NetworkToHostOrder(
                                                       BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueY)) / 100000;
                        torqueZ = (double)IPAddress.NetworkToHostOrder(
                                                       BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueZ)) / 100000;
                        break;
                    case SensorType.NewOptoForce:
                    case SensorType.OnRobot:
                        forceX = (double)IPAddress.NetworkToHostOrder(
                                                    BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceX)) / 10000;
                        forceY = (double)IPAddress.NetworkToHostOrder(
                                                    BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceY)) / 10000;
                        forceZ = (double)IPAddress.NetworkToHostOrder(
                                                    BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.ForceZ)) / 10000;
                        torqueX = (double)IPAddress.NetworkToHostOrder(
                                                       BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueX)) / 100000;
                        torqueY = (double)IPAddress.NetworkToHostOrder(
                                                       BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueY)) / 100000;
                        torqueZ = (double)IPAddress.NetworkToHostOrder(
                                                       BitConverter.ToInt32(getDatas, (int)ForceDatasMarks.TorqueZ)) / 100000;
                        break;
                    default:
                        forceX = 0.0;
                        forceY = 0.0;
                        forceZ = 0.0;
                        torqueX = 0.0;
                        torqueY = 0.0;
                        torqueZ = 0.0;
                        break;
                }
            }
        }

        /// <summary>
        /// 切换力传感器的数据清零模式，阻塞式
        /// </summary>
        /// <param name="SwitchState">是否清零数据</param>
        /// <param name="IntervalTime">开关间隔时间，默认12ms</param>
        public void SwitchBiasOpenOrClose(bool SwitchState = false, int IntervalTime = 12)
        {
            SendBiasCommand(false);
            if (SwitchState)
            {
                Task tempTask = Task.Run(new Action(() =>
                                                   {
                                                       Thread.Sleep(IntervalTime);
                                                       SendBiasCommand(true);
                                                       Thread.Sleep(IntervalTime);
                                                   }));
                tempTask.Wait();
            }
        }

        /// <summary>
        /// 返回采集到的力信号
        /// </summary>
        /// <returns></returns>
        public virtual double[] GetOrignalForceInformation()
        {
            double[] returnforces;
            lock (lockedObject)
            {
                returnforces = new double[] { forceX, forceY, forceZ, torqueX, torqueY, torqueZ };
            }
            return returnforces;
        }

        /// <summary>
        /// OPTO通讯连接中断，必须重载
        /// </summary>
        protected virtual void OPTOConnectionBroken() { }

        /// <summary>
        /// 关闭49152端口通讯，必须重载
        /// </summary>
        public virtual void Close49152Client()
        {
            CloseClient();
        }
        #endregion


    }
}
