using System.Net;
using System.Net.Sockets;

namespace URCommunication
{
    /// <summary>
    /// TCP通讯基类
    /// </summary>
    public class TCPBase
    {
        #region 枚举
        /// <summary>
        /// Socket状态
        /// </summary>
        protected enum SocketStatus : short
        {
            UnCreated = 0,
            Created,
            Connected,
            Closed
        }
        #endregion

        #region 字段
        protected string remoteIP = "192.168.1.1"; // 通讯对方IP地址
        protected int remotePort = 29999; // 通讯对方端口号 
        protected int socketTimeout = 500; // 收发超时时间
        protected bool ifTimeOutLooseAtBegin = false; // 起始段是否放大超时时间

        protected Socket remoteSocket = null; // Socket对象
        protected IPEndPoint remoteIpe = null; // IPE对象
        protected SocketStatus remoteSocketStatus = SocketStatus.UnCreated; // Socket对象状态

        protected const int looseMaxCount = 5; // 端口监听前几个周期放大超时时间
        protected const int looseMaxProp = 5; // 端口监听前几个周期超时时间放大倍数
        protected int looseCount = 0; // 大超时时间监听个数
        #endregion

        #region 属性
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
        public int RemotePort
        {
            get { return remotePort; }
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
        /// <param name="IP">远程IP地址</param>
        /// <param name="Port">远程端口号</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="IfTimeOutLooseAtBegin">起始段是否放大超时时间</param>
        protected void CreatClient(string IP, int Port, int TimeOut, bool IfTimeOutLooseAtBegin = false)
        {
            remoteIP = IP;
            remotePort = Port;
            socketTimeout = TimeOut;
            ifTimeOutLooseAtBegin = IfTimeOutLooseAtBegin;
            remoteSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            // 通讯超时时间
            if (ifTimeOutLooseAtBegin)
            {
                remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout, socketTimeout * looseMaxProp);
                remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, socketTimeout * looseMaxProp);
                looseCount = 0;
            }
            else
            {
                remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout, socketTimeout);
                remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, socketTimeout);
            }

            remoteIpe = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);

            if (remoteSocketStatus == SocketStatus.UnCreated)
            {
                remoteSocketStatus = SocketStatus.Created;
            }

            // 只在socket实例化的时候连接一次
            remoteSocket.Connect(remoteIpe);
            remoteSocketStatus = SocketStatus.Connected;
        }

        /// <summary>
        /// 发送指令
        /// </summary>
        /// <param name="BufferSend">要发送的指令</param>
        protected void SendCommand(byte[] BufferSend)
        {
            remoteSocket.Send(BufferSend);
        }

        /// <summary>
        /// 接收数据
        /// </summary>
        /// <param name="ByteLength">要接收的指令长度</param>
        /// <returns>返回接收到的数据</returns>
        protected byte[] RecieveDatas(int ByteLength)
        {
            byte[] recievedbytes = new byte[ByteLength];
            int recievedbyteslen = remoteSocket.Receive(recievedbytes);
            if (recievedbyteslen == ByteLength)
            {
                if (ifTimeOutLooseAtBegin)
                {
                    if (looseCount < looseMaxCount)
                    {
                        looseCount++;
                    }
                    if (looseCount == looseMaxCount)
                    {
                        looseCount++;
                        remoteSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, socketTimeout);
                    }
                }

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
