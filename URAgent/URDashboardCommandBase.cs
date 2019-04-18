// No using namespace

namespace URCommand
{
    /// <summary>
    /// URDashBoard指令集基类
    /// </summary>
    public class URDashboardCommandBase
    {
        /// <summary>
        /// 机械臂上电
        /// </summary>
        /// <returns>上电字符串</returns>
        public string PowerOn()
        {
            return "power on\n";
        }

        /// <summary>
        /// 机械臂解除抱闸
        /// </summary>
        /// <returns>解除抱闸字符串</returns>
        public string BrakeRelease()
        {
            return "brake release\n";
        }

        /// <summary>
        /// 机械臂断电
        /// </summary>
        /// <returns>断电字符串</returns>
        public string PowerOff()
        {
            return "power off\n";
        }

        /// <summary>
        /// 机械臂断电同时控制箱关机
        /// </summary>
        /// <returns>关机字符串</returns>
        public string ShutDown()
        {
            return "shutdown\n";
        }

        /// <summary>
        /// 立即停止机械臂运动程序
        /// </summary>
        /// <returns>停止程序字符串</returns>
        public string Stop()
        {
            return "stop\n";
        }
    }
}
