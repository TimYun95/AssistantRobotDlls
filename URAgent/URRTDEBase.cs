using System;
using System.Collections.Generic;
using System.Net;

namespace URCommunication
{
    /// <summary>
    /// UR中的RTDE通讯模块基类
    /// </summary>
    public class URRTDEBase : TCPBase
    {
        #region 枚举
        /// <summary>
        /// RTDE包类型
        /// </summary>
        protected enum RTDEPackageType : byte
        {
            RtdeControlPackageSetupInputs = 73,
            RtdeTextMessage = 77,
            RtdeControlPackageSetupOutputs = 79,
            RtdeControlPackagePause = 80,
            RtdeControlPackageStart = 83,
            RtdeDataPackage = 85,
            RtdeRequestProtocolVersion = 86,
            RtdeGetURControlVersion = 118
        }
        #endregion

        #region 字段
        protected byte recipeID = 0; // 包传递ID
        #endregion

        #region 方法
        /// <summary>
        /// 创建RTDE通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="IfLoose">是否放松起始超时时间</param>
        /// <param name="Port">远程端口号，默认30004</param>
        protected void CreatRTDEClient(string IP, int TimeOut, bool IfLoose, int Port = 30004)
        {
            CreatClient(IP, Port, TimeOut, IfLoose);
        }

        /// <summary>
        /// 发送输入寄存器设置指令
        /// </summary>
        /// <param name="RequiredParams">所需要的参数，格式见手册</param>
        protected virtual void SendInputSetup(string RequiredParams)
        {
            AddHeadToSend((byte)RTDEPackageType.RtdeControlPackageSetupInputs, RequiredParams);

            RecieveRecipeID();
        }

        /// <summary>
        /// 发送需要输入寄存器的参数
        /// </summary>
        /// <param name="InputDatas">输入参数</param>
        protected virtual void SendInputDatas(byte[] InputDatas)
        {
            List<byte> Content = new List<byte>(255);

            // 为输入参数增加包传递ID
            Content.Add(recipeID);
            Content.AddRange(InputDatas);

            // 发送输入参数
            AddHeadToSend((byte)RTDEPackageType.RtdeDataPackage, Content.ToArray());
        }

        /// <summary>
        /// 添加消息头后发送消息
        /// </summary>
        /// <param name="PackageType">包类型</param>
        /// <param name="Content">包内容</param>
        protected virtual void AddHeadToSend(byte PackageType, string Content)
        {
            List<byte> Package = new List<byte>(255);

            // 添加消息头 包长度
            byte[] PackageSize = BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)(Content.Length + 3)));
            
            Package.Add(PackageSize[0]);
            Package.Add(PackageSize[1]);

            // 添加消息头 包类型
            Package.Add(PackageType);

            // 添加消息内容
            byte[] PackageContent = System.Text.Encoding.Default.GetBytes(Content);
            Package.AddRange(PackageContent);

            // 发送包
            byte[] EntirePackage = Package.ToArray();
            SendCommand(EntirePackage);
        }

        /// <summary>
        /// 添加消息头后发送消息
        /// </summary>
        /// <param name="PackageType">包类型</param>
        /// <param name="Content">包内容</param>
        protected virtual void AddHeadToSend(byte PackageType, byte[] Content)
        {
            List<byte> Package = new List<byte>(255);

            // 添加消息头 包长度
            byte[] PackageSize = BitConverter.GetBytes(IPAddress.HostToNetworkOrder((Int16)(Content.Length + 3)));

            Package.Add(PackageSize[0]);
            Package.Add(PackageSize[1]);

            // 添加消息头 包类型
            Package.Add(PackageType);

            // 添加消息内容
            Package.AddRange(Content);

            // 发送包
            byte[] EntirePackage = Package.ToArray();
            SendCommand(EntirePackage);
        }

        /// <summary>
        /// 接收得到的包传递ID并保存
        /// </summary>
        protected virtual void RecieveRecipeID()
        {
            byte[] recievedbytes = new byte[100];
            int recievedbyteslen = remoteSocket.Receive(recievedbytes);
            recipeID = recievedbytes[3];
        }

        /// <summary>
        /// 关闭RTDE端口通讯
        /// </summary>
        protected void CloseRTDEClient()
        {
            CloseClient();
        }
        #endregion
    }
}
