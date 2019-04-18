using System.Collections.Generic;
using URCommand;

namespace URCommunication
{
    /// <summary>
    /// UR中的30002端口通讯类
    /// </summary>
    public class UR30002Connector : URTCPBase
    {
        #region 字段
        public URScriptBase scriptCommand; // 脚本指令
        protected bool ifHanged; // 安装方式
        protected int voltageLevel; // 工具电压
        protected double[] tcpCordinate; // TCP坐标
        protected double toolGravity; // 工具重量
        protected double toolBaryCenterX; // 工具重心X坐标
        protected double toolBaryCenterY; // 工具重心Y坐标
        protected double toolBaryCenterZ; // 工具重心Z坐标
        #endregion

        #region 属性
        /// <summary>
        /// 安装方式
        /// </summary>
        public bool IfHanged
        {
            get { return ifHanged; }
            set { ifHanged = value; }
        }

        /// <summary>
        /// 工具电压
        /// </summary>
        public int VoltageLevel
        {
            get { return voltageLevel; }
            set { voltageLevel = value; }
        }

        /// <summary>
        /// TCP坐标
        /// </summary>
        public double[] TCPCordinate
        {
            get { return (double[])tcpCordinate.Clone(); }
            set { tcpCordinate = (double[])value.Clone(); }
        }

        /// <summary>
        /// 工具重量
        /// </summary>
        public double ToolGravity
        {
            get { return toolGravity; }
            set { toolGravity = value; }
        }

        /// <summary>
        /// 工具重心X坐标
        /// </summary>
        public double ToolBaryCenterX
        {
            get { return toolBaryCenterX; }
            set { toolBaryCenterX = value; }
        }

        /// <summary>
        /// 工具重心Y坐标
        /// </summary>
        public double ToolBaryCenterY
        {
            get { return toolBaryCenterY; }
            set { toolBaryCenterY = value; }
        }

        /// <summary>
        /// 工具重心Z坐标
        /// </summary>
        public double ToolBaryCenterZ
        {
            get { return toolBaryCenterZ; }
            set { toolBaryCenterZ = value; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="HangedOrNot">是否倒挂</param>
        /// <param name="DigitalVoltage">工具电压</param>
        /// <param name="TcpCartesianPoint">工具坐标</param>
        /// <param name="GravityOfTool">工具重量kg</param>
        /// <param name="XCordinateForToolMass">工具重心x坐标，默认0m</param>
        /// <param name="YCordinateForToolMass">工具重心y坐标，默认0m</param>
        /// <param name="ZCordinateForToolMass">工具重心z坐标，默认0m</param>
        public UR30002Connector(bool HangedOrNot, int DigitalVoltage, double[] TcpCartesianPoint, double GravityOfTool, double XCordinateForToolMass = 0, double YCordinateForToolMass = 0, double ZCordinateForToolMass = 0)
        {
            scriptCommand = new URScriptBase();
            ifHanged = HangedOrNot;
            voltageLevel = DigitalVoltage;
            tcpCordinate = (double[])TcpCartesianPoint.Clone();
            toolGravity = GravityOfTool;
            toolBaryCenterX = XCordinateForToolMass;
            toolBaryCenterY = YCordinateForToolMass;
            toolBaryCenterZ = ZCordinateForToolMass;
        }

        /// <summary>
        /// 创建30002端口通讯并连接
        /// </summary>
        /// <param name="IP">远程IP地址</param>
        /// <param name="TimeOut">收发超时时间</param>
        /// <param name="Port">远程端口号，默认30002</param>
        public void Creat30002Client(string IP, int TimeOut, int Port = 30002)
        {
            CreatClient(IP, Port, TimeOut);
        }

        /// <summary>
        /// 发送工具线性移动指令
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendMoveL(double[] AimToolPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            List<string> movelStr = new List<string>(255);
            movelStr.Add(scriptCommand.SetGravity(ifHanged));
            movelStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            movelStr.Add(scriptCommand.SetTCP(tcpCordinate));
            movelStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            movelStr.Add(scriptCommand.MoveL(AimToolPosition, a, v, t, r));

            SendStringCommand(scriptCommand.MotionBlockFun(movelStr));
        }

        /// <summary>
        /// 发送工具线性移动指令，输入关节角度值
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendMoveLViaJ(double[] AimJointPosition, double a = 1.2, double v = 0.25, double t = 0, double r = 0)
        {
            List<string> movelviajStr = new List<string>(255);
            movelviajStr.Add(scriptCommand.SetGravity(ifHanged));
            movelviajStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            movelviajStr.Add(scriptCommand.SetTCP(tcpCordinate));
            movelviajStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            movelviajStr.Add(scriptCommand.MoveLViaJ(AimJointPosition, a, v, t, r));

            SendStringCommand(scriptCommand.MotionBlockFun(movelviajStr));
        }

        /// <summary>
        /// 发送关节线性移动指令
        /// </summary>
        /// <param name="AimJointPosition">目标关节数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendMoveJ(double[] AimJointPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            List<string> movejStr = new List<string>(255);
            movejStr.Add(scriptCommand.SetGravity(ifHanged));
            movejStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            movejStr.Add(scriptCommand.SetTCP(tcpCordinate));
            movejStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            movejStr.Add(scriptCommand.MoveJ(AimJointPosition, a, v, t, r));

            SendStringCommand(scriptCommand.MotionBlockFun(movejStr));
        }

        /// <summary>
        /// 发送关节线性移动指令，输入目标位置值
        /// </summary>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="v">移动速度，默认1.05rad/s</param>
        /// <param name="t">移动花费时间，默认不设置</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendMoveJViaL(double[] AimToolPosition, double a = 1.4, double v = 1.05, double t = 0, double r = 0)
        {
            List<string> movejStr = new List<string>(255);
            movejStr.Add(scriptCommand.SetGravity(ifHanged));
            movejStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            movejStr.Add(scriptCommand.SetTCP(tcpCordinate));
            movejStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            movejStr.Add(scriptCommand.MoveJViaL(AimToolPosition, a, v, t, r));

            SendStringCommand(scriptCommand.MotionBlockFun(movejStr));
        }

        /// <summary>
        /// 发送工具圆周运动指令
        /// </summary>
        /// <param name="ViaToolPosition">途径位置数组</param>
        /// <param name="AimToolPosition">目标位置数组</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="v">移动速度，默认0.25m/s</param>
        /// <param name="r">移动交融半径，默认0m</param>
        public void SendMoveC(double[] ViaToolPosition, double[] AimToolPosition, double a = 1.2, double v = 0.25, double r = 0)
        {
            List<string> movecStr = new List<string>(255);
            movecStr.Add(scriptCommand.SetGravity(ifHanged));
            movecStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            movecStr.Add(scriptCommand.SetTCP(tcpCordinate));
            movecStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            movecStr.Add(scriptCommand.MoveC(ViaToolPosition, AimToolPosition, a, v, r));

            SendStringCommand(scriptCommand.MotionBlockFun(movecStr));
        }

        /// <summary>
        /// 发送工具定向移动指令
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组，包括方向</param>
        /// <param name="a">移动加速度，默认1.2m/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        public void SendSpeedL(double[] VelocityDirection, double a = 1.2, double t = 60)
        {
            List<string> speedlStr = new List<string>(255);
            speedlStr.Add(scriptCommand.SetGravity(ifHanged));
            speedlStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            speedlStr.Add(scriptCommand.SetTCP(tcpCordinate));
            speedlStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            speedlStr.Add(scriptCommand.SpeedL(VelocityDirection, a, t));

            SendStringCommand(scriptCommand.MotionBlockFun(speedlStr));
        }

        /// <summary>
        /// 发送关节定向移动指令
        /// </summary>
        /// <param name="VelocityDirection">移动速度数组，包括方向</param>
        /// <param name="a">移动加速度，默认1.4rad/s^2</param>
        /// <param name="t">移动总时间，默认60s，即自动停止</param>
        public void SendSpeedJ(double[] VelocityDirection, double a = 1.4, double t = 60)
        {
            List<string> speedjStr = new List<string>(255);
            speedjStr.Add(scriptCommand.SetGravity(ifHanged));
            speedjStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            speedjStr.Add(scriptCommand.SetTCP(tcpCordinate));
            speedjStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            speedjStr.Add(scriptCommand.SpeedJ(VelocityDirection, a, t));

            SendStringCommand(scriptCommand.MotionBlockFun(speedjStr));
        }

        /// <summary>
        /// 发送工具线性停止指令
        /// </summary>
        /// <param name="a">制动加速度，默认1.2m/s^2</param>
        public void SendStopL(double a = 1.2)
        {
            List<string> stoplStr = new List<string>(255);
            stoplStr.Add(scriptCommand.SetGravity(ifHanged));
            stoplStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            stoplStr.Add(scriptCommand.SetTCP(tcpCordinate));
            stoplStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            stoplStr.Add(scriptCommand.StopL(a));

            SendStringCommand(scriptCommand.MotionBlockFun(stoplStr));
        }

        /// <summary>
        /// 发送开始反驱示教模式
        /// </summary>
        /// <param name="DurationTime">示教模式持续时间，默认3600s</param>
        public void SendBeginTeachMode(int DurationTime = 3600)
        {
            List<string> teachModeStr = new List<string>(255);
            teachModeStr.Add(scriptCommand.SetGravity(ifHanged));
            teachModeStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            teachModeStr.Add(scriptCommand.SetTCP(tcpCordinate));
            teachModeStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            teachModeStr.Add(scriptCommand.TeachMode());
            teachModeStr.Add(scriptCommand.Pause(DurationTime));

            SendStringCommand(scriptCommand.MotionBlockFun(teachModeStr));
        }

        /// <summary>
        /// 发送停止反驱示教模式
        /// </summary>
        public void SendEndTeachMode()
        {
            List<string> teachModeStopStr = new List<string>(255);
            teachModeStopStr.Add(scriptCommand.SetGravity(ifHanged));
            teachModeStopStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            teachModeStopStr.Add(scriptCommand.SetTCP(tcpCordinate));
            teachModeStopStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            teachModeStopStr.Add(scriptCommand.EndTeachMode());

            SendStringCommand(scriptCommand.MotionBlockFun(teachModeStopStr));
        }

        /// <summary>
        /// 发送暂停指令
        /// </summary>
        /// <param name="DurationTime">暂停时间，单位s</param>
        public void SendPause(int DurationTime)
        {
            List<string> pauseStr = new List<string>(255);
            pauseStr.Add(scriptCommand.SetGravity(ifHanged));
            pauseStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            pauseStr.Add(scriptCommand.SetTCP(tcpCordinate));
            pauseStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
            pauseStr.Add(scriptCommand.Pause(DurationTime));

            SendStringCommand(scriptCommand.MotionBlockFun(pauseStr));
        }

        /// <summary>
        /// 发送控制程序
        /// </summary>
        public void SendControllerCode()
        {
            SendStringCommand(scriptCommand.ControllerCode(ifHanged, voltageLevel, tcpCordinate, toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));
        }

        /// <summary>
        /// 发送自定义运动指令
        /// </summary>
        /// <param name="MotionCommand">自定义运动指令字符串</param>
        public void SendMotionCommand(List<string> MotionCommand)
        {
            List<string> motionStr = new List<string>(255);
            motionStr.Add(scriptCommand.SetGravity(ifHanged));
            motionStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            motionStr.Add(scriptCommand.SetTCP(tcpCordinate));
            motionStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));

            foreach (string lineStr in MotionCommand)
            {
                motionStr.Add(lineStr + "\n");
            }

            SendStringCommand(scriptCommand.MotionBlockFun(motionStr));
        }

        /// <summary>
        /// 发送自定义非运动指令
        /// </summary>
        /// <param name="NonMotionCommand">自定义非运动指令字符串</param>
        public void SendNonMotionCommand(List<string> NonMotionCommand)
        {
            List<string> nonMotionStr = new List<string>(255);
            nonMotionStr.Add(scriptCommand.SetGravity(ifHanged));
            nonMotionStr.Add(scriptCommand.SetToolVoltage(voltageLevel));
            nonMotionStr.Add(scriptCommand.SetTCP(tcpCordinate));
            nonMotionStr.Add(scriptCommand.SetPayLoad(toolGravity, toolBaryCenterX, toolBaryCenterY, toolBaryCenterZ));

            foreach (string lineStr in NonMotionCommand)
            {
                nonMotionStr.Add(lineStr + "\n");
            }

            SendStringCommand(scriptCommand.NonMotionBlockFun(nonMotionStr));
        }

        /// <summary>
        /// 发送基础设置
        /// </summary>
        public void SendBaseSetting()
        {
            SendStopL();
        }

        /// <summary>
        /// 关闭30002端口通讯
        /// </summary>
        public void Close30002Client()
        {
            CloseClient();
        }
        #endregion

    }
}
