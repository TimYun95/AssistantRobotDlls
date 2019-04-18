using URCommand;

namespace URCommunication
{
    /// <summary>
    /// UR中的29999端口通讯类
    /// </summary>
    public class UR29999Connector : URTCPBase
    {
        #region 字段
        protected URDashboardCommandBase dashboardCommand; // DashBoard指令
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        public UR29999Connector()
        {
            dashboardCommand = new URDashboardCommandBase();
        }

        /// <summary>
        /// 创建29999端口通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="Port">远程端口号，默认29999</param>
        public void Creat29999Client(string IP, int TimeOut, int Port = 29999)
        {
            CreatClient(IP, Port, TimeOut);
        }

        /// <summary>
        /// 发送机械臂上电指令
        /// </summary>
        public void SendPowerOn()
        {
            string powerOnStr = dashboardCommand.PowerOn();
            SendStringCommand(powerOnStr);
        }

        /// <summary>
        /// 发送机械臂开闸指令
        /// </summary>
        public void SendBrakeRelease()
        {
            string brakeReleaseStr = dashboardCommand.BrakeRelease();
            SendStringCommand(brakeReleaseStr);
        }

        /// <summary>
        /// 发送机械臂断电指令
        /// </summary>
        public void SendPowerOff()
        {
            string powerOffStr = dashboardCommand.PowerOff();
            SendStringCommand(powerOffStr);
        }

        /// <summary>
        /// 发送机械臂和控制箱关机指令
        /// </summary>
        public void SendShutDown()
        {
            string shutDownStr = dashboardCommand.ShutDown();
            SendStringCommand(shutDownStr);
        }

        /// <summary>
        /// 发送机械臂停止运行运动程序指令
        /// </summary>
        public void SendStop()
        {
            string stopStr = dashboardCommand.Stop();
            SendStringCommand(stopStr);
        }

        /// <summary>
        /// 关闭29999端口通讯
        /// </summary>
        public void Close29999Client()
        {
            CloseClient();
        }
        #endregion
    }
}
