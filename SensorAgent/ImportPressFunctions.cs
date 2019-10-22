using System;
//using System.Collections.Generic;
using System.Linq;
//using System.Text;
using System.Threading.Tasks;
using System.Management;
using System.IO.Ports;

using LogPrinter;

namespace SensorAgent
{
    class ImportPressFunctions
    {
        #region 字段
        protected static string port;
        protected static int baudRate = 115200; // 波特率
        protected static Parity parityBit = Parity.None; // 校验位
        protected static int dataBitLength = 8; // 数据位
        protected static StopBits stopBitLength = StopBits.One; // 停止位
        protected static SerialPort ComDevice = new SerialPort(); // 串口实例
        protected static string serialDeviceName = "CH340";
        protected static bool isDeviceConnected = false;
        protected static bool isPortBeListened = false;
        protected static bool isPortListeningEnd = false;

        protected static bool isDataFailed = false;

        protected static int[] recDatas = new int[4];

        private static readonly object datalocker = new object(); // 数据锁
        #endregion

        #region 方法
        /// <summary>
        /// 无公开构造
        /// </summary>
        private ImportPressFunctions() { }

        public static bool AskDeviceExist()
        {
            string queryPort = "Fail to found";

            using (ManagementObjectSearcher searcher = new ManagementObjectSearcher("select * from Win32_PnPEntity"))
            {
                var hardInfos = searcher.Get();
                foreach (var hardInfo in hardInfos)
                {
                    object name = hardInfo.Properties["Name"].Value;
                    if (name != null && name.ToString().Contains("CH340"))
                    {
                        int index = name.ToString().LastIndexOf("COM");
                        int indexEnd = name.ToString().LastIndexOf(")");

                        queryPort = name.ToString().Substring(index, indexEnd - index);
                    }
                }
            }

            if (queryPort == "Fail to found") return false;

            port = queryPort;
            return true;
        }

        public static bool ConnectToSensor()
        {
            // 若已经连接上，则返回true
            if (isDeviceConnected) return true;

            // 若设备不存在，则返回false
            if (!AskDeviceExist()) return false;

            // 若未连接，且设备存在，则初始化并连接
            // 设置串口相关属性
            ComDevice.PortName = port;
            ComDevice.BaudRate = baudRate;
            ComDevice.Parity = parityBit;
            ComDevice.DataBits = dataBitLength;
            ComDevice.StopBits = stopBitLength;

            // 打开串口
            ComDevice.Open();
            isDeviceConnected = true;

            // 循环监听
            Task.Run(new Action(ListenFromPort));
            return true;
        }

        public static bool DisconnectFromSensor()
        {
            // 若未连接上，则返回true
            if (!isDeviceConnected) return true;

            // 只要连接上了，无论设备是否存在都断开
            isPortBeListened = false;
            while (!isPortListeningEnd) ;
            ComDevice.Close();

            return true;
        }

        public static double[] GetInfo()
        {
            bool temp = true;
            int[] recDataCopy = new int[4];
            lock (datalocker)
            {
                if (isDataFailed) temp = false;
                else recDataCopy = (int[])recDatas.Clone();
            }

            if (!temp) return null;
            double[] giveOutDatas = new double[4];
            for (int i = 0; i < 4; ++i) giveOutDatas[i] = recDataCopy[i];
            return giveOutDatas;
        }

        protected static void ListenFromPort()
        {
            isPortBeListened = true;
            isPortListeningEnd = false;

            while (isPortBeListened)
            {
                try
                {
                    lock (datalocker)
                    {
                        FetchDatasFromBuffer();
                    }
                }
                catch (Exception ex)
                {
                    isPortBeListened = false;
                    Logger.HistoryPrinting(Logger.Level.ERROR, "Inside sensor dll", "Fail to get datas from pressure sensor", ex);
                }
            }

            isPortBeListened = false;
            isPortListeningEnd = true;
        }

        protected static void FetchDatasFromBuffer()
        {
            if (ComDevice.BytesToRead < 12)
                return;
            byte[] readBuffer = new byte[12];
            ComDevice.Read(readBuffer, 0, 12);

            // Adjust buffer datas and refresh datas
            int headIndex = FindHead(readBuffer);
            if (headIndex < 0)
            {
                isDataFailed = true;
                return;
            }
            else if (headIndex > 0)
            {
                int moreLen = headIndex;
                byte[] newBuffer = new byte[12];
                readBuffer.Skip(headIndex).ToArray().CopyTo(newBuffer, 0);
                while (ComDevice.BytesToRead < moreLen) ;
                ComDevice.Read(newBuffer, 12 - headIndex, moreLen);
                if (!CheckSum(newBuffer))
                {
                    isDataFailed = true;
                    return;
                }
                RefreshRecDatas(newBuffer);
                isDataFailed = false;
            }
            else
            {
                if (!CheckSum(readBuffer))
                {
                    isDataFailed = true;
                    return;
                }
                RefreshRecDatas(readBuffer);
                isDataFailed = false;
            }
        }

        protected static int FindHead(byte[] array)
        {
            for (int i = 0; i < 12; ++i)
            {
                int current = i;
                int next = (i + 1 > 11) ? (i + 1 - 11 - 1) : (i + 1);
                int next2 = (next + 1 > 11) ? (next + 1 - 11 - 1) : (next + 1);

                if (array[current] == 255 && array[next] == 0 && array[next2] == 0)
                {
                    return current;
                }
            }
            return -1;
        }

        protected static bool CheckSum(byte[] array)
        {
            byte sum = 0;
            // The checksum shoud be test to find
            for (int i = 1; i < 11; ++i) sum += array[i];
            if (sum == array[11]) return true;
            return false;
        }

        protected static void RefreshRecDatas(byte[] thisBuffer)
        {
            recDatas[0] = thisBuffer[3] * 255 + thisBuffer[4];
            recDatas[1] = thisBuffer[5] * 255 + thisBuffer[6];
            recDatas[2] = thisBuffer[7] * 255 + thisBuffer[8];
            recDatas[3] = thisBuffer[9] * 255 + thisBuffer[10];
        }

        #endregion
    }
}
