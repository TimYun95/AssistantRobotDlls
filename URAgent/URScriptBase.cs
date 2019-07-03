using System;
using System.Collections.Generic;
using System.IO;

namespace URCommand
{
    /// <summary>
    /// UR脚本指令集基类
    /// </summary>
    public class URScriptBase
    {
        #region 字段
        protected string controllerCodePath = AppDomain.CurrentDomain.BaseDirectory + "ControllerCode\\ControllerCode.txt"; // 下位机实时控制代码保存地址
        #endregion

        #region 属性
        /// <summary>
        /// 下位机实时控制代码保存地址
        /// </summary>
        public string ControllerCodePath
        {
            get { return controllerCodePath; }
            set { controllerCodePath = value; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 工具线性移动
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string MoveL(double[] AimToolPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            string str;
            if (t != 0 && r != 0)
            {
                str = "movel(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                            + AimToolPosition[1].ToString("0.0000") + ", "
                                            + AimToolPosition[2].ToString("0.0000") + ", "
                                            + AimToolPosition[3].ToString("0.000") + ", "
                                            + AimToolPosition[4].ToString("0.000") + ", "
                                            + AimToolPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000")
                                            + ", t = " + t.ToString("0.000")
                                            + ", r = " + r.ToString("0.0000") + ")";
            }
            else if (t != 0 && r == 0)
            {
                str = "movel(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                            + AimToolPosition[1].ToString("0.0000") + ", "
                                            + AimToolPosition[2].ToString("0.0000") + ", "
                                            + AimToolPosition[3].ToString("0.000") + ", "
                                            + AimToolPosition[4].ToString("0.000") + ", "
                                            + AimToolPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000")
                                            + ", t = " + t.ToString("0.000") + ")";
            }
            else if (t == 0 && r != 0)
            {
                str = "movel(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                            + AimToolPosition[1].ToString("0.0000") + ", "
                                            + AimToolPosition[2].ToString("0.0000") + ", "
                                            + AimToolPosition[3].ToString("0.000") + ", "
                                            + AimToolPosition[4].ToString("0.000") + ", "
                                            + AimToolPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000")
                                            + ", r = " + r.ToString("0.0000") + ")";
            }
            else
            {
                str = "movel(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                            + AimToolPosition[1].ToString("0.0000") + ", "
                                            + AimToolPosition[2].ToString("0.0000") + ", "
                                            + AimToolPosition[3].ToString("0.000") + ", "
                                            + AimToolPosition[4].ToString("0.000") + ", "
                                            + AimToolPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000") + ")";
            }
            str += "\n";
            return str;
        }

        /// <summary>
        /// 工具线性移动，输入关节角度值
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string MoveLViaJ(double[] AimJointPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            string str;
            if (t != 0 && r != 0)
            {
                str = "movel([" + AimJointPosition[0].ToString("0.000") + ", "
                                            + AimJointPosition[1].ToString("0.000") + ", "
                                            + AimJointPosition[2].ToString("0.000") + ", "
                                            + AimJointPosition[3].ToString("0.000") + ", "
                                            + AimJointPosition[4].ToString("0.000") + ", "
                                            + AimJointPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000")
                                            + ", t = " + t.ToString("0.000")
                                            + ", r = " + r.ToString("0.0000") + ")";
            }
            else if (t != 0 && r == 0)
            {
                str = "movel([" + AimJointPosition[0].ToString("0.000") + ", "
                                            + AimJointPosition[1].ToString("0.000") + ", "
                                            + AimJointPosition[2].ToString("0.000") + ", "
                                            + AimJointPosition[3].ToString("0.000") + ", "
                                            + AimJointPosition[4].ToString("0.000") + ", "
                                            + AimJointPosition[5].ToString("0.000")
                                            + "], a = " + a.ToString("0.000")
                                            + ", v = " + v.ToString("0.000")
                                            + ", t = " + t.ToString("0.000") + ")";
            }
            else if (t == 0 && r != 0)
            {
                str = "movel([" + AimJointPosition[0].ToString("0.000") + ", "
                                                            + AimJointPosition[1].ToString("0.000") + ", "
                                                            + AimJointPosition[2].ToString("0.000") + ", "
                                                            + AimJointPosition[3].ToString("0.000") + ", "
                                                            + AimJointPosition[4].ToString("0.000") + ", "
                                                            + AimJointPosition[5].ToString("0.000")
                                                            + "], a = " + a.ToString("0.000")
                                                            + ", v = " + v.ToString("0.000")
                                                            + ", r = " + r.ToString("0.0000") + ")";
            }
            else
            {
                str = "movel([" + AimJointPosition[0].ToString("0.000") + ", "
                                                            + AimJointPosition[1].ToString("0.000") + ", "
                                                            + AimJointPosition[2].ToString("0.000") + ", "
                                                            + AimJointPosition[3].ToString("0.000") + ", "
                                                            + AimJointPosition[4].ToString("0.000") + ", "
                                                            + AimJointPosition[5].ToString("0.000")
                                                            + "], a = " + a.ToString("0.000")
                                                            + ", v = " + v.ToString("0.000") + ")";
            }
            str += "\n";
            return str;
        }

        /// <summary>
        /// 关节线性移动
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string MoveJ(double[] AimJointPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            string str;
            if (t != 0 && r != 0)
            {
                str = "movej([" + AimJointPosition[0].ToString("0.000") + ", "
                                          + AimJointPosition[1].ToString("0.000") + ", "
                                          + AimJointPosition[2].ToString("0.000") + ", "
                                          + AimJointPosition[3].ToString("0.000") + ", "
                                          + AimJointPosition[4].ToString("0.000") + ", "
                                          + AimJointPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", t = " + t.ToString("0.000")
                                          + ", r = " + r.ToString("0.0000") + ")";
            }
            else if (t != 0 && r == 0)
            {
                str = "movej([" + AimJointPosition[0].ToString("0.000") + ", "
                                          + AimJointPosition[1].ToString("0.000") + ", "
                                          + AimJointPosition[2].ToString("0.000") + ", "
                                          + AimJointPosition[3].ToString("0.000") + ", "
                                          + AimJointPosition[4].ToString("0.000") + ", "
                                          + AimJointPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", t = " + t.ToString("0.000") + ")";
            }
            else if (t == 0 && r != 0)
            {
                str = "movej([" + AimJointPosition[0].ToString("0.000") + ", "
                                          + AimJointPosition[1].ToString("0.000") + ", "
                                          + AimJointPosition[2].ToString("0.000") + ", "
                                          + AimJointPosition[3].ToString("0.000") + ", "
                                          + AimJointPosition[4].ToString("0.000") + ", "
                                          + AimJointPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", r = " + r.ToString("0.0000") + ")";
            }
            else
            {
                str = "movej([" + AimJointPosition[0].ToString("0.000") + ", "
                                          + AimJointPosition[1].ToString("0.000") + ", "
                                          + AimJointPosition[2].ToString("0.000") + ", "
                                          + AimJointPosition[3].ToString("0.000") + ", "
                                          + AimJointPosition[4].ToString("0.000") + ", "
                                          + AimJointPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000") + ")";
            }
            str += "\n";
            return str;
        }
      
        /// <summary>
        /// 关节线性移动，输入目标位置值
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string MoveJViaL(double[] AimToolPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            string str;
            if (t != 0 && r != 0)
            {
                str = "movej(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                          + AimToolPosition[1].ToString("0.0000") + ", "
                                          + AimToolPosition[2].ToString("0.0000") + ", "
                                          + AimToolPosition[3].ToString("0.000") + ", "
                                          + AimToolPosition[4].ToString("0.000") + ", "
                                          + AimToolPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", t = " + t.ToString("0.000")
                                          + ", r = " + r.ToString("0.0000") + ")";
            }
            else if (t != 0 && r == 0)
            {
                str = "movej(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                          + AimToolPosition[1].ToString("0.0000") + ", "
                                          + AimToolPosition[2].ToString("0.0000") + ", "
                                          + AimToolPosition[3].ToString("0.000") + ", "
                                          + AimToolPosition[4].ToString("0.000") + ", "
                                          + AimToolPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", t = " + t.ToString("0.000") + ")";
            }
            else if (t == 0 && r != 0)
            {
                str = "movej(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                          + AimToolPosition[1].ToString("0.0000") + ", "
                                          + AimToolPosition[2].ToString("0.0000") + ", "
                                          + AimToolPosition[3].ToString("0.000") + ", "
                                          + AimToolPosition[4].ToString("0.000") + ", "
                                          + AimToolPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000")
                                          + ", r = " + r.ToString("0.0000") + ")";
            }
            else
            {
                str = "movej(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                          + AimToolPosition[1].ToString("0.0000") + ", "
                                          + AimToolPosition[2].ToString("0.0000") + ", "
                                          + AimToolPosition[3].ToString("0.000") + ", "
                                          + AimToolPosition[4].ToString("0.000") + ", "
                                          + AimToolPosition[5].ToString("0.000")
                                          + "], a = " + a.ToString("0.000")
                                          + ", v = " + v.ToString("0.000") + ")";
            }
            str += "\n";
            return str;
        }

        /// <summary>
        /// 工具圆周运动
        /// </summary>
        /// <param name="ViaToolPosition">途径位置数组</param>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="r">移动交融半径，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string MoveC(double[] ViaToolPosition, double[] AimToolPosition, double a = 1.2, double v = 0.25, double r = 0)
        {
            string str = "movec(p[" + ViaToolPosition[0].ToString("0.0000") + ", "
                                                   + ViaToolPosition[1].ToString("0.0000") + ", "
                                                   + ViaToolPosition[2].ToString("0.0000") + ", "
                                                   + ViaToolPosition[3].ToString("0.000") + ", "
                                                   + ViaToolPosition[4].ToString("0.000") + ", "
                                                   + ViaToolPosition[5].ToString("0.000") + "], "
                                        + "p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                                   + AimToolPosition[1].ToString("0.0000") + ", "
                                                   + AimToolPosition[2].ToString("0.0000") + ", "
                                                   + AimToolPosition[3].ToString("0.000") + ", "
                                                   + AimToolPosition[4].ToString("0.000") + ", "
                                                   + AimToolPosition[5].ToString("0.000") + "]"
                                                   + ", a = " + a.ToString("0.000")
                                                   + ", v = " + v.ToString("0.000")
                                                   + ", r = " + r.ToString("0.0000") + ")";
            str += "\n";
            return str;
        }

        /// <summary>
        /// 伺服关节移动
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="t">伺服移动时间，默认0.008s</param>
        /// <param name="lookahead_time">预计移动时间，默认0.1s</param>
        /// <param name="gain">比例增益，默认300</param>
        /// <returns>返回脚本字符串</returns>
        public string ServoJ(double[] AimToolPosition, double t = 0.008, double lookahead_time = 0.1, int gain = 300)
        {
            string str = "servoj(get_inverse_kin(p[" + AimToolPosition[0].ToString("0.0000") + ", "
                                                                              + AimToolPosition[1].ToString("0.0000") + ", "
                                                                              + AimToolPosition[2].ToString("0.0000") + ", "
                                                                              + AimToolPosition[3].ToString("0.000") + ", "
                                                                              + AimToolPosition[4].ToString("0.000") + ", "
                                                                              + AimToolPosition[5].ToString("0.000")
                                                                              + "]), t = " + t.ToString("0.000")
                                                                              + ", lookahead_time = " + lookahead_time.ToString("0.00")
                                                                              + ", gain = " + gain.ToString("0") + ")";
            str += "\n";
            return str;
        }

        /// <summary>
        /// 工具定向移动
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组，包括方向</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        /// <returns>返回脚本字符串</returns>
        public string SpeedL(double[] VelocityDirection, double a = 1.2, double t = 60)
        {
            string str = "speedl([" + VelocityDirection[0].ToString("0.0000") + ", "
                                                 + VelocityDirection[1].ToString("0.0000") + ", "
                                                 + VelocityDirection[2].ToString("0.0000") + ", "
                                                 + VelocityDirection[3].ToString("0.000") + ", "
                                                 + VelocityDirection[4].ToString("0.000") + ", "
                                                 + VelocityDirection[5].ToString("0.000")
                                                 + "], a = " + a.ToString()
                                                 + ", t = " + t.ToString() + ")";
            str += "\n";
            return str;
        }

        /// <summary>
        /// 关节定向移动
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组，包括方向</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        /// <returns>返回脚本字符串</returns>
        public string SpeedJ(double[] VelocityDirection, double a = 1.4, double t = 60)
        {
            string str = "speedj([" + VelocityDirection[0].ToString("0.0000") + ", "
                                                 + VelocityDirection[1].ToString("0.0000") + ", "
                                                 + VelocityDirection[2].ToString("0.0000") + ", "
                                                 + VelocityDirection[3].ToString("0.0000") + ", "
                                                 + VelocityDirection[4].ToString("0.0000") + ", "
                                                 + VelocityDirection[5].ToString("0.0000")
                                                 + "], a = " + a.ToString()
                                                 + ", t = " + t.ToString() + ")";
            str += "\n";
            return str;
        }

        /// <summary>
        /// 工具线性停止
        /// </summary>
        /// <param name="a">制动加速度，默认1.2m/s^2</param>
        /// <returns>返回脚本字符串</returns>
        public string StopL(double a = 1.2)
        {
            string str = "stopl(a = " + a.ToString("0.000") + ")\n";
            return str;
        }

        /// <summary>
        /// 设置安装位形，只有正放和倒挂两种
        /// </summary>
        /// <param name="IfHanged">是否倒挂</param>
        /// <returns>返回脚本字符串</returns>
        public string SetGravity(bool IfHanged)
        {
            if (!IfHanged)
            {
                string str = "set_gravity([0, 0, 9.82])\n";
                return str;
            }
            else
            {
                string str = "set_gravity([0, 0, -9.82])\n";
                return str;
            }
        }

        /// <summary>
        /// 设置工具位置和姿态参数
        /// </summary>
        /// <param name="TcpCordinate">工具坐标</param>
        /// <returns>返回脚本字符串</returns>
        public string SetTCP(double[] TcpCordinate)
        {
            string str = "set_tcp(p[" + TcpCordinate[0].ToString("0.0000") + ", "
                                                    + TcpCordinate[1].ToString("0.0000") + ", "
                                                    + TcpCordinate[2].ToString("0.0000") + ", "
                                                    + TcpCordinate[3].ToString("0.000") + ", "
                                                    + TcpCordinate[4].ToString("0.000") + ", "
                                                    + TcpCordinate[5].ToString("0.000") + "])\n";
            return str;
        }

        /// <summary>
        /// 设置工具重量和重心
        /// </summary>
        /// <param name="ToolGravity">工具重量kg</param>
        /// <param name="ToolBaryCenterX">工具重心x坐标，默认0m</param>
        /// <param name="ToolBaryCenterY">工具重心y坐标，默认0m</param>
        /// <param name="ToolBaryCenterZ">工具重心z坐标，默认0m</param>
        /// <returns>返回脚本字符串</returns>
        public string SetPayLoad(double ToolGravity, double ToolBaryCenterX = 0, double ToolBaryCenterY = 0, double ToolBaryCenterZ = 0)
        {
            if (ToolBaryCenterX == 0 && ToolBaryCenterY == 0 && ToolBaryCenterZ == 0)
            {
                string str = "set_payload(" + ToolGravity.ToString("0.00") + ")\n";
                return str;
            }
            else
            {
                string str = "set_payload(" + ToolGravity.ToString("0.00") +
                                   ", [" + ToolBaryCenterX.ToString("0.0000") +
                                   ", " + ToolBaryCenterY.ToString("0.0000") +
                                   ", " + ToolBaryCenterZ.ToString("0.0000") + "])\n";
                return str;
            }
        }

        /// <summary>
        /// 设置工具口电压
        /// </summary>
        /// <param name="VoltageLevel">电压，只有12V和24V</param>
        /// <returns>返回脚本字符串</returns>
        public string SetToolVoltage(int VoltageLevel)
        {
            if (VoltageLevel == 12 || VoltageLevel == 24)
            {
                return "set_tool_voltage(" + VoltageLevel.ToString("0") + ")\n";
            }
            else
            {
                return "set_tool_voltage(0)\n";
            }
        }

        /// <summary>
        /// 打开示教模式
        /// </summary>
        /// <returns>返回脚本字符串</returns>
        public string TeachMode()
        {
            return "teach_mode()\n";
        }

        /// <summary>
        /// 关闭示教模式
        /// </summary>
        /// <returns>返回脚本字符串</returns>
        public string EndTeachMode()
        {
            return "end_teach_mode()\n";
        }

        /// <summary>
        /// 暂停一段时间
        /// </summary>
        /// <param name="Seconds">暂停时间，单位s</param>
        /// <returns>返回脚本字符串</returns>
        public string Pause(int Seconds)
        {
            return "sleep(" + Seconds.ToString("0") + ")\n";
        }

        /// <summary>
        /// 运动块状脚本程序
        /// </summary>
        /// <param name="Content">程序主要内容数组</param>
        /// <returns>返回脚本字符串</returns>
        public string MotionBlockFun(List<string> Content)
        {
            string str = "def MotionFun():\n";

            foreach (string linecontent in Content)
            {
                str += ("  " + linecontent);
            }

            str += "end\n";
            return str;
        }

        /// <summary>
        /// 非运动块状脚本程序
        /// </summary>
        /// <param name="Content">程序主要内容数组</param>
        /// <returns>返回脚本字符串</returns>
        public string NonMotionBlockFun(List<string> Content)
        {
            string str = "sec NonMotionFun():\n";

            foreach (string linecontent in Content)
            {
                str += ("  " + linecontent);
            }

            str += "end\n";
            return str;
        }

        /// <summary>
        /// 控制箱执行程序
        /// </summary>
        /// <param name="IfHanged">是否倒挂</param>
        /// <param name="VoltageLevel">电压，只有12V和24V</param>
        /// <param name="TcpCordinate">工具坐标</param>
        /// <param name="ToolGravity">工具重量kg</param>
        /// <param name="ToolBaryCenterX">工具重心x坐标，默认0m</param>
        /// <param name="ToolBaryCenterY">工具重心y坐标，默认0m</param>
        /// <param name="ToolBaryCenterZ">工具重心z坐标，默认0m</param>
        /// <returns>返回控制箱执行程序</returns>
        public string ControllerCode(bool IfHanged, int VoltageLevel, double[] TcpCordinate, double ToolGravity, double ToolBaryCenterX = 0, double ToolBaryCenterY = 0, double ToolBaryCenterZ = 0)
        {
            string[] streamReaderContent;
            using (StreamReader streamReader = File.OpenText(controllerCodePath))
            {
                streamReaderContent = streamReader.ReadToEnd().Split(new string[] { "\r\n" }, StringSplitOptions.RemoveEmptyEntries);
            }

            List<string> strList = new List<string>(255);
            strList.Add(SetGravity(IfHanged));
            strList.Add(SetToolVoltage(VoltageLevel));
            strList.Add(SetTCP(TcpCordinate));
            strList.Add(SetPayLoad(ToolGravity, ToolBaryCenterX, ToolBaryCenterY, ToolBaryCenterZ));

            foreach (string lineStr in streamReaderContent)
            {
                strList.Add(lineStr + "\n");
            }

            return MotionBlockFun(strList);
        }



        #endregion
    }
}
