using System;
using System.IO.Ports;
using System.Reflection;
using System.Timers;
using LogPrinter;

namespace SerialConnection
{
    /// <summary>
    /// 串口连接基类
    /// </summary>
    public class SerialBase
    {
        #region 字段
        protected string portName = ""; // COM号
        protected int baudRate = 0; // 波特率
        protected Parity parityBit = Parity.None; // 校验位
        protected int dataBitLength = 0; // 数据位
        protected StopBits stopBitLength = StopBits.One; // 停止位
        protected SerialPort ComDevice = new SerialPort(); // 串口实例

        protected int reccurentInterval = 0; // 重复检验周期，单位ms
        public delegate void SendVoid(); // 无参数发送委托

        /// <summary>
        /// 发送COM连接失效消息
        /// </summary>
        public event SendVoid OnSendCOMInvalid;

        Timer recurrentCheckTimer = new Timer(); // 循环检验COM口定时器
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="ConnectedPort">连接的COM口</param>
        /// <param name="TransferRate">波特率，默认9600bps</param>
        /// <param name="CheckMode">校验方式，默认无</param>
        /// <param name="DataUnitLength">数据单元长度，默认8</param>
        /// <param name="StopBitLength">停止位长度，默认1</param>
        /// <param name="RecurrenInterval">重复检验周期，默认5000ms</param>
        public SerialBase(string ConnectedPort, int TransferRate = 9600, Parity CheckMode = Parity.None, int DataUnitLength = 8, StopBits StopBitLength = StopBits.One, int RecurrenInterval = 5000)
        {
            portName = ConnectedPort;
            baudRate = TransferRate;
            parityBit = CheckMode;
            dataBitLength = DataUnitLength;
            stopBitLength = StopBitLength;
            reccurentInterval = RecurrenInterval;

            // 设置串口相关属性
            ComDevice.PortName = portName;
            ComDevice.BaudRate = baudRate;
            ComDevice.Parity = parityBit;
            ComDevice.DataBits = dataBitLength;
            ComDevice.StopBits = stopBitLength;

            // 设置定时器
            recurrentCheckTimer.AutoReset = true;
            recurrentCheckTimer.Interval = reccurentInterval;
            recurrentCheckTimer.Elapsed += new ElapsedEventHandler(RecurrentCheckingWork);
        }

        /// <summary>
        /// 打开COM连接
        /// </summary>
        public void OpenCOMConnection()
        {
            if (!ComDevice.IsOpen)
            {
                // 打开串口
                try
                {
                    ComDevice.Open();
                    recurrentCheckTimer.Start();
                }
                catch (Exception ex)
                {
                    if (ComDevice.IsOpen)
                    {
                        CloseCOMConnection();
                    }
                    OnSendCOMInvalid();
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "COM message cannot be sent.", ex);
                }
            }
        }

        /// <summary>
        /// 向COM发送消息
        /// </summary>
        /// <param name="Data"></param>
        protected void SendDataToCOM(byte[] Data)
        {
            try
            {
                //将消息传递给串口
                ComDevice.Write(Data, 0, Data.Length);
            }
            catch (Exception ex)
            {
                if (ComDevice.IsOpen)
                {
                    CloseCOMConnection();
                }
                OnSendCOMInvalid();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "COM message cannot be sent.", ex);
            }
        }

        /// <summary>
        /// 关闭COM连接
        /// </summary>
        public void CloseCOMConnection()
        {
            if (ComDevice.IsOpen)
            {
                recurrentCheckTimer.Stop();
                ComDevice.Close();
            }
        }

        /// <summary>
        /// 检验定时器触发程序
        /// </summary>
        protected virtual void RecurrentCheckingWork(object obj, ElapsedEventArgs args)
        {
            if (!ComDevice.IsOpen)
            {
                OnSendCOMInvalid();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "COM closed unexpectly.");
                recurrentCheckTimer.Stop();
            }
        }
        #endregion
    }
}
