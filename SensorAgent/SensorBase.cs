using System;
using System.Threading;
using System.Threading.Tasks;
using System.Reflection;

using LogPrinter;

namespace SensorAgent
{
    /// <summary>
    /// 传感器数据读取基类
    /// </summary>
    public class SensorBase
    {
        #region 字段
        protected bool isDeviceConnected = false;
        protected System.Timers.Timer dataTimer = new System.Timers.Timer();
        protected bool isInsideTimer = false;
        protected int count = 0;
        protected bool insideLoopCrash = false;
        protected bool isAlsoInsideTimer = false;

        protected double[] infos = new double[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        private static readonly object infoLocker = new object();

        protected System.Timers.Timer sendTimer = new System.Timers.Timer();

        public delegate void SendDoubleArray(double[] array); // 发送double数组委托
        public delegate void SendVoid(); // 发送无参数委托

        /// <summary>
        /// 发送传感器采集的数据，数据无效时返回null
        /// </summary>
        public event SendDoubleArray OnSendSenorDatas;

        /// <summary>
        /// 发送传感器采集数据错误，需要从外部断连
        /// </summary>
        public event SendVoid OnSendSenorDataWrong;

        protected bool ifUseGyroSensor = true;
        protected bool ifUseInfraredSensor = true;
        protected bool ifUsePressSensor = true;

        protected int sendDataPeriod = 56;
        protected int getDataPeriod = 25;
        #endregion

        #region 方法
        /// <summary>
        /// SensorBase构造
        /// </summary>
        /// <param name="useInfrared">是否使用红外线传感器，默认使用</param>
        /// <param name="useGyro">是否使用陀螺仪传感器，默认使用</param>
        /// <param name="usePress">是否使用压电传感器，默认使用</param>
        /// <param name="innerPeriod">内部周期，默认25 ms</param>
        /// <param name="pushPeriod">推送周期，默认 56 ms</param>
        public SensorBase(bool useInfrared = true, bool useGyro = true,
                                      bool usePress = true, int innerPeriod = 25, int pushPeriod = 56)
        {
            ifUseGyroSensor = useGyro;
            ifUseInfraredSensor = useInfrared;
            ifUsePressSensor = usePress;

            sendDataPeriod = pushPeriod;
            getDataPeriod = innerPeriod;
        }

        /// <summary>
        /// 检查是否存在传感器
        /// </summary>
        /// <returns>返回结果</returns>
        public bool CheckIfDevicesAllExist()
        {
            if (ifUseInfraredSensor && !ImportInfraredFunctions.AskDeviceExist())
                return false;
            if (ifUseGyroSensor && !ImportGyroFunctions.AskDeviceExist())
                return false;
            if (ifUsePressSensor && !ImportPressFunctions.AskDeviceExist())
                return false;
            if (!ifUseInfraredSensor && !ifUseGyroSensor && !ifUsePressSensor)
                return false;

            return true;
        }

        /// <summary>
        /// 连接设备
        /// </summary>
        /// <returns>返回连接结果</returns>
        public bool ConnectDevices()
        {
            // 设备已经连接，直接返回true
            if (isDeviceConnected) return true;

            // 设备不存在，直接返回false
            bool existResult = CheckIfDevicesAllExist();
            if (!existResult) return false;

            // 连接Infrared
            bool[] connectedResult = { false, false, false };
            if (ifUseInfraredSensor)
                connectedResult[0] = ImportInfraredFunctions.ConnectToSensor();

            // 连接Gyro
            if (ifUseGyroSensor)
                connectedResult[1] = ImportGyroFunctions.ConnectToSensor();

            // 连接Press
            if (ifUsePressSensor)
                connectedResult[2] = ImportPressFunctions.ConnectToSensor();

            // 处理Press连接结果
            bool connectReceipt = true;
            if (ifUsePressSensor && !connectedResult[2])
            {
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when connect to press sensor.");
                connectReceipt = false;
            }

            // 处理Gyro连接结果
            if (ifUseGyroSensor && !connectedResult[1])
            {
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when connect to gyro sensor.");
                connectReceipt = false;
            }

            // 处理Infrared连接结果
            if (ifUseInfraredSensor && !connectedResult[0])
            {
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when connect to infrared sensor.");
                connectReceipt = false;
            }

            if (!connectReceipt) return false;

            dataTimer.Interval = getDataPeriod;
            dataTimer.AutoReset = true;
            dataTimer.Elapsed += new System.Timers.ElapsedEventHandler(RefreshInfoElapsed);
            dataTimer.Start();
            insideLoopCrash = false;

            Task.Run(new Action(() =>
            {
                Thread.Sleep(200);
                sendTimer.Interval = sendDataPeriod;
                sendTimer.AutoReset = true;
                sendTimer.Elapsed += new System.Timers.ElapsedEventHandler(SendInfoElapsed);
                sendTimer.Start();
            }));

            isDeviceConnected = true;
            return true;
        }

        /// <summary>
        /// 断开设备
        /// </summary>
        /// <returns>返回断开结果</returns>
        public bool DisconnectDevices()
        {
            if (!isDeviceConnected) return true;

            int timeOutCount = 0;
            dataTimer.Stop();
            while (isInsideTimer && timeOutCount < 10)
                timeOutCount++;
            if (timeOutCount >= 10)
                return false;
            timeOutCount = 0;
            Thread.Sleep(100);

            insideLoopCrash = true;

            sendTimer.Stop();
            while (isAlsoInsideTimer && timeOutCount < 10)
                timeOutCount++;
            if (timeOutCount >= 10)
                return false;
            timeOutCount = 0;
            Thread.Sleep(100);

            bool[] killResult = { false, false, false };

            if (ifUseInfraredSensor)
            {
                killResult[0] = ImportInfraredFunctions.DisconnectFromSensor();
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when disconnect from infrared sensor.");
            }

            if (ifUseGyroSensor)
            {
                killResult[1] = ImportGyroFunctions.DisconnectFromSensor();
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when disconnect from gyro sensor.");
            }

            if (ifUsePressSensor)
            {
                killResult[2] = ImportPressFunctions.DisconnectFromSensor();
                Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when disconnect from press sensor.");
            }

            if (ifUseInfraredSensor && !killResult[0]) return false;
            if (ifUseGyroSensor && !killResult[1]) return false;
            if (ifUsePressSensor && !killResult[2]) return false;

            isDeviceConnected = false;
            return true;
        }

        /// <summary>
        /// 更新数据
        /// </summary>
        protected void RefreshInfoElapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (isInsideTimer) return;
            isInsideTimer = true;

            double[] infoFromInfrared = ifUseInfraredSensor ? ImportInfraredFunctions.GetInfo() : null;
            double[] infoFromGyro = ifUseGyroSensor ? ImportGyroFunctions.GetInfo() : null;
            double[] infoFromPress = ifUsePressSensor ? ImportPressFunctions.GetInfo() : null;

            if ((ifUseInfraredSensor && infoFromInfrared == null) ||
                (ifUseGyroSensor && infoFromGyro == null) ||
                (ifUsePressSensor && infoFromPress == null))
            {
                count++;
                isInsideTimer = false;
                return;
            }
             
            count = 0;
            if (count > 10)
            {
                insideLoopCrash = true;
                dataTimer.Stop();
                sendTimer.Stop();

                Task.Run(new Action(() =>
                {
                    OnSendSenorDataWrong();
                }));

                isInsideTimer = false;
                return;
            }

            lock (infoLocker)
            {
                if (ifUseInfraredSensor)
                {
                    infos[0] = infoFromInfrared[0];
                    infos[1] = infoFromInfrared[1];
                }

                if (ifUseGyroSensor)
                {
                    infos[2] = infoFromGyro[0];
                    infos[3] = infoFromGyro[1];
                    infos[4] = infoFromGyro[2];
                    infos[5] = infoFromGyro[3];
                }

                if (ifUsePressSensor)
                {
                    infos[6] = infoFromPress[0];
                    infos[7] = infoFromPress[1];
                    infos[8] = infoFromPress[2];
                    infos[9] = infoFromPress[3];
                }
            }

            isInsideTimer = false;
        }

        /// <summary>
        /// 发送数据
        /// </summary>
        protected void SendInfoElapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (isAlsoInsideTimer) return;
            isAlsoInsideTimer = true;

            if (insideLoopCrash)
            {
                OnSendSenorDatas(null);

                isAlsoInsideTimer = false;
                return;
            }

            double[] tempDatas = new double[10];
            lock (infoLocker)
            {
                tempDatas = (double[])infos.Clone();
            }
            OnSendSenorDatas(tempDatas);

            isAlsoInsideTimer = false;
        }

        #endregion
    }






}
