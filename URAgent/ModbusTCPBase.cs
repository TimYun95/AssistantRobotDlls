using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;

namespace URCommunication
{
    /// <summary>
    /// ModbusTCP通讯基类
    /// </summary>
    public class ModbusTCPBase
    {
        #region 枚举
        /// <summary>
        /// ModbusTCP通讯功能码
        /// </summary>
        protected enum FunctionCode : byte
        {
            Read = 3,
            Write = 16
        }
        #endregion

        #region 字段
        protected string remoteIP = "192.168.1.1"; // 通讯对方IP地址
        protected int remotePort = 502; // 通讯对方端口号 
        protected int socketTimeout = 500; // 收发超时时间

        protected Socket modbusSocket = null; // Socket对象
        protected IPEndPoint modbusIpe = null; // IPE对象

        //写多个寄存器，则不同寄存器的值以“|”分开
        protected char regDivider = '|';
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

        /// <summary>
        /// 寄存器分隔符
        /// </summary>
        public char RegisterDivider
        {
            get { return regDivider; }
        }
        #endregion

        #region 方法
        /// <summary> 
        /// 初始化通讯，初始化一次，以后直接保持连接
        /// </summary>
        /// <param name="IP">对方IP地址</param>
        /// <param name="Port">对方端口号</param>
        /// <param name="TimeOut">收发超时时间</param>
        protected void InitialServer(string IP, int Port, int TimeOut)
        {
            // 设置Socket基本信息
            remoteIP = IP;
            remotePort = Port;
            socketTimeout = TimeOut;
            modbusSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            // 设置这个Socket的收发超时时间
            modbusSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout, socketTimeout);
            modbusSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, socketTimeout);

            // 用同步方法进行连接
            modbusIpe = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);

            // 只在socket实例化的时候连接一次
            modbusSocket.Connect(modbusIpe);
        }

        /// <summary>
        /// 写寄存器
        /// </summary>
        /// <param name="RegisterString">写入寄存器的字符串，多个值用分隔符“|”隔开</param>
        /// <param name="StartAddress">首个寄存器地址</param>
        /// <returns>返回写寄存器是否成功</returns>
        protected virtual void WriteMultipleRegister(string RegisterString, int StartAddress)
        {
            // 将传入的字符串按RegisterDivision进行分割
            string[] RegisterStringArray = RegisterString.Split(regDivider);

            // 获得写入的寄存器个数
            int RegisterNum = RegisterStringArray.Length;

            // 把分割好的字符串存入byte数组(写1个寄存器要2个byte，因为UR是16bit的寄存器)
            byte[] data = new byte[RegisterNum * 2];

            // 把分割好的每个寄存器要写入的值依次添加进来
            for (int i = 0; i < RegisterStringArray.Length; i++)
            {
                //把每个数字提取到tempArray里面
                byte[] tempArray = IntTobyteArray(Convert.ToUInt16(RegisterStringArray[i]));
                data[i * 2] = tempArray[0];
                data[i * 2 + 1] = tempArray[1];
            }

            List<byte> values = new List<byte>(255);

            //（数据位：1-2） 定义我是谁(Transaction Identifier),协议给了我2byte空间定义我是谁（我代号就是01，如果Modbus TCP Server接收成功，返回信息也要有这个代号）
            values.AddRange(new Byte[] { 0, 1 });

            //（数据位：3-4）定义协议号，协议给了我2byte空间定义协议号（协议号就是00，表示这是MODBUS 协议）
            values.AddRange(new Byte[] { 0, 0 });

            //（数据位：5-6）现在考虑将56两位凑到一块
            byte[] DataByte = IntTobyteArray(Convert.ToUInt16(data.Length + 7));
            values.Add(DataByte[0]);
            values.Add(DataByte[1]);

            //（数据位：7）只要补全一个0即可
            values.Add(0);

            //（数据位：8）定义功能码（把写多个寄存器这个16码转换为byte类型数据）Function Code : 16 (Write Multiple Register)
            values.Add((byte)FunctionCode.Write);

            //（数据位：9-10）起始地址，现在把地址这个int类型转换为两个byte
            byte[] AddressByte = IntTobyteArray(Convert.ToUInt16(StartAddress));
            values.Add(AddressByte[0]);
            values.Add(AddressByte[1]);

            //（数据位：11-12）寄存器数量（小于255个）
            values.Add(0);
            values.Add((byte)RegisterNum);

            //（数据位：13）发送数据的长度，跟前面保持不变
            values.Add((byte)data.Length);

            // 添加发送的数据
            values.AddRange(data);

            // 发送数据
            modbusSocket.Send(values.ToArray());

            // 立即等待返回
            int ReadBuffer = 64;
            byte[] ReadBufferData = new byte[ReadBuffer];
            modbusSocket.Receive(ReadBufferData);

            // 返回数据表示发送是否成功
            if (ReadBufferData[0] > (byte)FunctionCode.Write)
            {
                throw new Exception("invalid sent datas on Modbus.");
            }
        }
        
        /// <summary>
        /// 读寄存器
        /// </summary>
        /// <param name="ResigterNum">寄存器个数</param>
        /// <param name="StartAddress">首个寄存器地址</param>
        /// <returns>返回读到的数据</returns>
        protected virtual int[] ReadMultipleRegister(int ResigterNum, int StartAddress)
        {
            // 读取的东西(byte是两个表示一个)
            byte[] TruelyDateByte = new byte[ResigterNum * 2];
            int[] TruelyDateInt = new int[ResigterNum];

            // 定义长度不确定的数组sendData，每个数组元素都是byte类型的整数
            List<byte> sendData = new List<byte>(255);

            //（数据位：1-2） 定义我是谁(Transaction Identifier),协议给了我2byte空间定义我是谁（我代号就是01，如果Modbus TCP Server接收成功，返回信息也要有这个代号）
            sendData.AddRange(new Byte[] { 0, 1 });

            //（数据位：3-4）定义协议号，协议给了我2byte空间定义协议号（协议号就是00，表示这是MODBUS 协议）
            sendData.AddRange(new Byte[] { 0, 0 });

            //（数据位：5-6）对于读取来说，header后面还有6
            byte[] DataByte = IntTobyteArray((UInt16)6);
            sendData.Add(DataByte[0]);
            sendData.Add(DataByte[1]);

            //（数据位：7）补全一个0
            sendData.Add(0);

            //（数据位：8）定义功能码（把读多个寄存器这个03码转换为byte类型数据）Function Code : 03 (Read Multiple Registers)
            sendData.Add((byte)FunctionCode.Read);

            //（数据位：9-10）起始地址，现在把地址这个int类型转换为两个byte
            byte[] AddressByte = IntTobyteArray(Convert.ToUInt16(StartAddress));
            sendData.Add(AddressByte[0]);
            sendData.Add(AddressByte[1]);

            //（数据位：11-12）寄存器数量，不超过255个
            sendData.Add(0);
            sendData.Add((byte)ResigterNum);

            // 发送查询命令
            modbusSocket.Send(sendData.ToArray());

            // 立即等待返回
            int ReadBuffer = 256;
            byte[] ReadBufferData = new byte[ReadBuffer];
            modbusSocket.Receive(ReadBufferData);
            // socket的Receive方法直接把读取到的数据返回给ReadBufferData
            //（0|1|0|0|0|9|0|3|6|）前面9位对读到的数据是无意义的
            for (int i = 0; i < TruelyDateByte.Length; i++)
            {
                TruelyDateByte[i] = ReadBufferData[i + 9];
            }

            // 然后byte再放回int
            for (int i = 0; i < TruelyDateInt.Length; i++)
            {
                TruelyDateInt[i] = TruelyDateByte[i * 2] * 256 + TruelyDateByte[i * 2 + 1];
            }

            return TruelyDateInt;
        }

        /// <summary>
        /// 将两个字节Int16类型数据转换为寄存器高低字节的方法，先高后低
        /// </summary>
        /// <param name="InputNum">输入两个字节的UInt16数据</param>
        /// <returns>返回转换得到的字节</returns>
        private byte[] IntTobyteArray(UInt16 InputNum)
        {
            byte[] temp = new byte[2];
            int InputNum_High = InputNum / 256;
            int InputNum_Low = InputNum % 256;

            //整数部分给高位，余数部分给低位
            temp[0] = (byte)InputNum_High;
            temp[1] = (byte)InputNum_Low;

            return temp;
        }

        /// <summary>
        /// 关闭Socket连接
        /// </summary>
        protected void CloseClient()
        {
            modbusSocket.Shutdown(SocketShutdown.Both);
            modbusSocket.Close();
        }
        #endregion
    }
}
