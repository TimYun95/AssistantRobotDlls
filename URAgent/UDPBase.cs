using System.Net;
using System.Net.Sockets;

namespace URCommunication
{
    /// <summary>
    /// UDP通讯基类
    /// </summary>
    public class UDPBase
    {
        #region 枚举
        /// <summary>
        /// Socket状态
        /// </summary>
        protected enum SocketStatus : short
        {
            UnCreated = 0,
            Created,
            Opened,
            Closed
        }
        #endregion

        #region 字段
        protected string localIP = "192.168.1.0"; // 通讯本地IP地址
        protected string remoteIP = "192.168.1.1"; // 通讯对方IP地址
        protected int publicPort = 49152; // 通讯公共端口号 
        protected int socketTimeout = 500; // 收发超时时间

        protected Socket remoteSocket = null; // Socket对象
        protected IPEndPoint remoteIpe = null; // IPE对象
        protected SocketStatus remoteSocketStatus = SocketStatus.UnCreated; // Socket对象状态
        #endregion

        #region 属性
        /// <summary>
        /// 通讯本地IP地址
        /// </summary>
        public string LocalIP
        {
            get { return localIP; }
        }

        /// <summary>
        /// 通讯对方IP地址
        /// </summary>
        public string RemoteIP
        {
            get { return remoteIP; }
        }

        /// <summary>
        /// 通讯对方端口号
        /// </summary>
        public int PublicPort
        {
            get { return publicPort; }
        }

        /// <summary>
        /// 收发超时时间
        /// </summary>
        public int SocketTimeOut
        {
            get { return socketTimeout; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 创建通讯并连接
        /// </summary>
        /// <param name="SelfIP">自身IP地址</param>
        /// <param name="OppositeIP">远程IP地址</param>
        /// <param name="Port">公共端口号</param>
        /// <param name="TimeOut">收发超时时间</param>
        protected void CreatClient(string SelfIP, string OppositeIP, int Port, int TimeOut)
        {
            localIP = SelfIP;
            remoteIP = OppositeIP;
            publicPort = Port;
            socketTimeout = TimeOut;
            remoteSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            // 通讯超时时间
            remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout, socketTimeout);
            remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, socketTimeout);

            remoteIpe = new IPEndPoint(IPAddress.Parse(remoteIP), publicPort);

            if (remoteSocketStatus == SocketStatus.UnCreated)
            {
                remoteSocketStatus = SocketStatus.Created;
            }

            // 只在socket实例化的时候进行绑定
            remoteSocket.Bind(new IPEndPoint(IPAddress.Parse(localIP), publicPort));

            remoteSocketStatus = SocketStatus.Opened;
        }

        /// <summary>
        /// 发送指令
        /// </summary>
        /// <param name="BufferSend">要发送的指令</param>
        protected void SendCommand(byte[] BufferSend)
        {
            remoteSocket.SendTo(BufferSend, (EndPoint)remoteIpe);
        }

        /// <summary>
        /// 接收数据
        /// </summary>
        /// <returns>返回接收到的数据</returns>
        protected byte[] RecieveDatas(int ByteLength)
        {
            byte[] recievedbytes = new byte[ByteLength];
            EndPoint RemoteE = (EndPoint)remoteIpe;

            int recievedbyteslen = remoteSocket.ReceiveFrom(recievedbytes, ref RemoteE);
            if (recievedbyteslen == ByteLength)
            {
                return recievedbytes;
            }
            else
            {
                return new byte[] { };
            }
        }

        /// <summary>
        /// 关闭Socket连接
        /// </summary>
        protected void CloseClient()
        {
            if (remoteSocketStatus != SocketStatus.UnCreated)
            {
                remoteSocket.Shutdown(SocketShutdown.Both);
                remoteSocket.Close();
                remoteSocketStatus = SocketStatus.Closed;
            }
        }
        #endregion
    }
}
