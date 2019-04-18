using System;
using System.Collections.Generic;
using System.Net;

namespace URCommunication
{
    /// <summary>
    /// UR中的30004端口通讯类
    /// </summary>
    public class UR30004Connector : URRTDEBase
    {
        #region 方法
        /// <summary>
        /// 创建30004通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="IfLoose">是否放松起始超时时间</param>
        /// <param name="Port">远程端口号，默认30004</param>
        public void Creat30004Client(string IP, int TimeOut, bool IfLoose, int Port = 30004)
        {
            CreatRTDEClient(IP, TimeOut, IfLoose, Port);
        }
        
        /// <summary>
        /// 设置寄存器用以存放发送的工具坐标
        /// </summary>
        /// <param name="inputSetupStr">需要使用的寄存器</param>
        public void ToolPositionInputSetup(string inputSetupStr = "input_double_register_0,input_double_register_1,input_double_register_2,input_double_register_3,input_double_register_4,input_double_register_5")
        {
            SendInputSetup(inputSetupStr);
        }

        /// <summary>
        /// 发送工具坐标到设置好的寄存器
        /// </summary>
        /// <param name="InputToolPosition">被发送的工具坐标</param>
        public void ToolPositionInputDatas(double[] InputToolPosition)
        {
            List<byte> byteList = new List<byte>(8 * InputToolPosition.Length);

            foreach (double inputValue in InputToolPosition)
            {
                byteList.AddRange(BitConverter.GetBytes(IPAddress.HostToNetworkOrder(BitConverter.DoubleToInt64Bits(inputValue))));
            }

            SendInputDatas(byteList.ToArray());
        }

        /// <summary>
        /// 关闭30004端口通讯
        /// </summary>
        public void Close30004Client()
        {
            CloseRTDEClient();
        }
        #endregion
    }
}
