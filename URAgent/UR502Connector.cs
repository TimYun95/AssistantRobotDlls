using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace URCommunication
{
    /// <summary>
    /// UR中的502端口Modbus通讯类
    /// </summary>
    public class UR502Connector : ModbusTCPBase
    {
        #region 字段
        protected const int sendSegment1MaxValue = 65535; // 发送段落1最大值
        protected const int sendSegment2MaxValue = 131070; // 发送段落2最大值
        protected const int sendSegment3MaxValue = 150000; // 发送段落整体最大值
        #endregion

        #region 方法
        /// <summary>
        /// 创建502通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="Port">远程端口号，默认30004</param>
        public void Creat502Client(string IP, int TimeOut, int Port = 502)
        {
            InitialServer(IP, Port, TimeOut);
        }

        /// <summary>
        /// 发送工具坐标到指定的寄存器中，连写15个，从第130号寄存器开始
        /// </summary>
        /// <param name="InputToolPosition">工具坐标数组，单位m，rad</param>
        public virtual void WriteRegister(double[] InputToolPosition)
        {
            int[] tcpX = PackOnePositionDouble(InputToolPosition[0]);
            int[] tcpY = PackOnePositionDouble(InputToolPosition[1]);
            int[] tcpZ = PackOnePositionDouble(InputToolPosition[2]);
            int[] tcpRX = PackOneAttitudeDouble(InputToolPosition[3]);
            int[] tcpRY = PackOneAttitudeDouble(InputToolPosition[4]);
            int[] tcpRZ = PackOneAttitudeDouble(InputToolPosition[5]);

            string str = tcpX[0].ToString("0") + "|" + tcpX[1].ToString("0") + "|" + tcpX[2].ToString("0") + "|" +
                              tcpY[0].ToString("0") + "|" + tcpY[1].ToString("0") + "|" + tcpY[2].ToString("0") + "|" +
                              tcpZ[0].ToString("0") + "|" + tcpZ[1].ToString("0") + "|" + tcpZ[2].ToString("0") + "|" +
                              tcpRX[0].ToString("0") + "|" + tcpRX[1].ToString("0") + "|" +
                              tcpRY[0].ToString("0") + "|" + tcpRY[1].ToString("0") + "|" +
                              tcpRZ[0].ToString("0") + "|" + tcpRZ[1].ToString("0");

            WriteMultipleRegister(str, 130);
        }

        /// <summary>
        /// 对表示位置的单个double变量进行打包，拆分成三个UInt16大小的变量
        /// </summary>
        /// <param name="Number">待打包的double变量</param>
        /// <returns>打包完毕的三个int变量</returns>
        protected virtual int[] PackOnePositionDouble(double Number)
        {
            int NumberValue = (int)Math.Round(Math.Abs(Number * 100000));
            int NumberSign = (-Math.Sign(Number) + 1) / 2 * 40000;
            int[] NumberSegment = { 0, 0, 0 };

            // 值拆分
            if (NumberValue > sendSegment1MaxValue)
            {
                NumberSegment[0] = sendSegment1MaxValue;
                if (NumberValue > sendSegment2MaxValue)
                {
                    NumberSegment[1] = sendSegment1MaxValue;
                    NumberSegment[2] = NumberValue - sendSegment2MaxValue;
                }
                else
                {
                    NumberSegment[1] = NumberValue - sendSegment1MaxValue;
                }
            }
            else
            {
                NumberSegment[0] = NumberValue;
            }

            // 符号拆分
            NumberSegment[2] += NumberSign;

            return NumberSegment;
        }

        /// <summary>
        /// 对表示姿态的单个double变量进行打包，拆分成两个UInt16大小的变量
        /// </summary>
        /// <param name="Number">待打包的double变量</param>
        /// <returns>打包完毕的两个int变量</returns>
        protected virtual int[] PackOneAttitudeDouble(double Number)
        {
            int NumberValue = (int)Math.Round(Math.Abs(Number * 10000));
            int NumberSign = (Math.Sign(Number) + 1) / 2;
            int[] NumberSegment = { NumberValue, NumberSign };

            return NumberSegment;
        }

        /// <summary>
        /// 关闭502端口通讯
        /// </summary>
        public void Close502Client()
        {
            CloseClient();
        }

        #endregion
    }
}
