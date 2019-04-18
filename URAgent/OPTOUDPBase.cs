using System;
using System.Net;

namespace URCommunication
{
    /// <summary>
    /// OPTO中的UDP基类
    /// </summary>
    public class OPTOUDPBase : UDPBase
    {
        #region 方法
        /// <summary>
        /// 发送指令，开始采集力信号，经过一定数量数据后停止
        /// </summary>
        /// <param name="SampleCount">采集数据量，输入0的话，让停的时候再停，默认0</param>
        protected virtual void SendStartCommand(int SampleCount = 0)
        {
            byte[] buffersend = new byte[8];
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x1234)), 0, buffersend, 0, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x0002)), 0, buffersend, 2, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(SampleCount)), 0, buffersend, 4, 4);
            SendCommand(buffersend);
        }

        /// <summary>
        /// 发送指令，设置是否传感器有偏置，实际作用是设置零点
        /// </summary>
        /// <param name="BiasOpen">是否有偏置，默认false</param>
        protected virtual void SendBiasCommand(bool BiasOpen = false)
        {
            byte[] buffersend = new byte[8];
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x1234)), 0, buffersend, 0, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x0042)), 0, buffersend, 2, 2);
            if (BiasOpen)
            {
                Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(255)), 0, buffersend, 4, 4);
            }
            else
            {
                Array.Copy(BitConverter.GetBytes(0), 0, buffersend, 4, 4);
            }
            SendCommand(buffersend);
        }

        /// <summary>
        /// 发送指令，设置截断频率
        /// </summary>
        /// <param name="Filter">频率标号，见手册，默认4，即15Hz</param>
        protected virtual void SendFilterCommand(int Filter = 4)
        {
            byte[] buffersend = new byte[8];
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x1234)), 0, buffersend, 0, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x0081)), 0, buffersend, 2, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(Filter)), 0, buffersend, 4, 4);
            SendCommand(buffersend);
        }

        /// <summary>
        /// 发送指令，设置采样频率，即读取数据的频率
        /// </summary>
        /// <param name="Frequency">采用频率，默认250Hz</param>
        protected virtual void SendFrequencyCommand(int Frequency = 250)
        {
            byte[] buffersend = new byte[8];
            if (Frequency > 500) Frequency = 500;
            else if (Frequency < 4) Frequency = 4;
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x1234)), 0, buffersend, 0, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x0082)), 0, buffersend, 2, 2);
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(1000 / Frequency)), 0, buffersend, 4, 4);
            SendCommand(buffersend);
        }

        /// <summary>
        /// 发送指令，停止采集数据
        /// </summary>
        protected virtual void SendStopCommand()
        {
            byte[] buffersend = new byte[8];
            Array.Copy(BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)0x1234)), 0, buffersend, 0, 2);
            Array.Copy(BitConverter.GetBytes((Int16)0x0000), 0, buffersend, 2, 2);
            Array.Copy(BitConverter.GetBytes(0), 0, buffersend, 4, 4);
            SendCommand(buffersend);
        }

        /// <summary>
        /// 读取用于实时获取传感器参数
        /// </summary>
        /// <returns>返回读到的字节数组</returns>
        protected byte[] Response(int ByteLength)
        {
            return RecieveDatas(ByteLength);
        }
        #endregion
    }
}
