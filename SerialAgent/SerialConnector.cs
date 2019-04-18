using System.IO.Ports;

namespace SerialConnection
{
    public class SerialConnector : SerialBase
    {
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
        public SerialConnector(string ConnectedPort, int TransferRate = 9600, Parity CheckMode = Parity.None, int DataUnitLength = 8, StopBits StopBitLength = StopBits.One, int RecurrenInterval = 5000)
            : base(ConnectedPort, TransferRate, CheckMode, DataUnitLength, StopBitLength, RecurrenInterval) { }

        /// <summary>
        /// 打开继电器
        /// </summary>
        public void SendOpenRelay()
        {
            SendDataToCOM(new byte[] { 160, 1, 1, 162 });
        }

        /// <summary>
        /// 关闭继电器
        /// </summary>
        public void SendCloseRelay()
        {
            SendDataToCOM(new byte[] { 160, 1, 0, 161 });
        }
        #endregion
    }
}
