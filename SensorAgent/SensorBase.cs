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

        protected double[] infos = new double[10];
        private static readonly object infoLocker = new object();

        protected System.Timers.Timer sendTimer = new System.Timers.Timer();

        public delegate void SendDoubleArray(double[] array); // 发送double数组委托

        /// <summary>
        /// 发送传感器采集的数据，数据无效时返回null
        /// </summary>
        public event SendDoubleArray OnSendSenorDatas;

        #endregion

        #region 方法
        /// <summary>
        /// 检查是否存在传感器
        /// </summary>
        /// <returns>返回结果</returns>
        public bool CheckIfDevicesAllExist()
        {
            //ImportInfraredFunctions.AskDeviceExist() &&
            //    ImportGyroFunctions.AskDeviceExist() &&
            if (
                ImportPressFunctions.AskDeviceExist())
                return true;
            else return false;
        }

        /// <summary>
        /// 连接设备
        /// </summary>
        /// <returns>返回连接结果</returns>
        public bool ConnectDevices()
        {
            if (isDeviceConnected) return true;

            bool existResult = CheckIfDevicesAllExist();
            if (!existResult) return false;

            bool connectedResult = ImportPressFunctions.ConnectToSensor();
            if (!connectedResult) return false;

            //bool connectedResult = ImportInfraredFunctions.ConnectToSensor();
            //if (!connectedResult) return false;

            //connectedResult = ImportGyroFunctions.ConnectToSensor();
            //if (!connectedResult)
            //{
            //    if (!ImportInfraredFunctions.DisconnectFromSensor())
            //        Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when connect to devices.");
            //    return false;
            //}

            //connectedResult = ImportPressFunctions.ConnectToSensor();
            //if (!connectedResult)
            //{
            //    if (!ImportInfraredFunctions.DisconnectFromSensor() || !ImportGyroFunctions.DisconnectFromSensor())
            //        Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Exception when connect to devices.");
            //    return false;
            //}

            dataTimer.Interval = 25;
            dataTimer.AutoReset = true;
            dataTimer.Elapsed += new System.Timers.ElapsedEventHandler(RefreshInfoElapsed);
            dataTimer.Start();
            insideLoopCrash = false;

            Task.Run(new Action(() =>
            {
                Thread.Sleep(200);
                sendTimer.Interval = 56;
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

            bool killResult = ImportPressFunctions.DisconnectFromSensor();
            if (!killResult) return false;

            //bool killResult = ImportInfraredFunctions.DisconnectFromSensor();
            //if (!killResult) return false;

            //killResult = ImportGyroFunctions.DisconnectFromSensor();
            //if (!killResult) return false;

            //killResult = ImportPressFunctions.DisconnectFromSensor();
            //if (!killResult) return false;

            dataTimer.Stop();
            insideLoopCrash = true;
            sendTimer.Stop();
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

            //double[] infoFromInfrared = ImportInfraredFunctions.GetInfo();
            //double[] infoFromGyro = ImportGyroFunctions.GetInfo();
            double[] infoFromPress = ImportPressFunctions.GetInfo();

            if (infoFromPress == null)
                //if (infoFromInfrared == null || infoFromGyro == null || infoFromPress == null)
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

                isInsideTimer = false;
                return;
            }

            lock (infoLocker)
            {
                //infos[0] = infoFromInfrared[0];
                //infos[1] = infoFromInfrared[1];
                //infos[2] = infoFromGyro[0];
                //infos[3] = infoFromGyro[1];
                //infos[4] = infoFromGyro[2];
                //infos[5] = infoFromGyro[3];
                infos[6] = infoFromPress[0];
                infos[7] = infoFromPress[1];
                infos[8] = infoFromPress[2];
                infos[9] = infoFromPress[3];
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
