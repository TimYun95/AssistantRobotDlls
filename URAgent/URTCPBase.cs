// No using namespace

namespace URCommunication
{
    /// <summary>
    /// UR中的TCP基类
    /// </summary>
    public class URTCPBase : TCPBase
    {
        #region 方法
        /// <summary>
        /// 发送指令
        /// </summary>
        /// <param name="Command">要发送的指令</param>
        protected void SendStringCommand(string Command)
        {
            byte[] buffersend = System.Text.Encoding.Default.GetBytes(Command);
            SendCommand(buffersend);
        }

        /// <summary>
        /// 监听返回的数据
        /// </summary>
        /// <param name="ByteLength">要接收的指令长度</param>
        /// <returns>返回读到的字节数组</returns>
        protected byte[] WaitForFeedback(int ByteLength)
        {
            return RecieveDatas(ByteLength);
        }

        /// <summary>
        /// 发送指令，并监听返回值
        /// </summary>
        /// <param name="Command">要发送的指令</param>
        /// <param name="ByteLength">要接收的指令长度</param>
        /// <returns>返回字符串</returns>
        protected byte[] SendStringCommandWithFeedback(string Command, int ByteLength)
        {
            SendStringCommand(Command);
            return WaitForFeedback(ByteLength);
        }
        #endregion
    }
}
