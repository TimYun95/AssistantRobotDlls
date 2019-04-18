using URCommunication;

namespace URServo
{
    /// <summary>
    /// 伺服运动模块基类
    /// </summary>`
    public abstract class ServoMotionBase
    {
        #region 枚举
        /// <summary>
        /// 伺服运动模块标志位
        /// </summary>
        protected enum ServoMotionModuleFlag : byte
        {
            BaseMotion = 0,
            FreeTranslation = 1,
            StraightTranslation = 2,
            TangentialTranslation = 3,
            SphereTranslation = 4
        }
        #endregion
        
        #region 字段
        protected URDataProcessor internalProcessor; // 内部UR数据处理类，用以获得相关数据交换和控制权
        protected bool ifPort30004Used = false; // 是否使用了30004端口

        protected double servoMotionControlPeriod = 0.0; // 伺服运动控制周期
        protected double servoMotionLookAheadTime = 0.0; // 伺服运动预计时间
        protected double servoMotionGain = 0.0; // 伺服运动增益

        protected bool servoMotionIfOpen = false; // 伺服运动模式是否打开
        protected int servoMotionOpenRound = 0; // 伺服运动模式打开的周期轮数
        protected const int servoMotionInitialRound = 5; // 伺服运动模式初始化周期轮数

        protected double[] servoMotionToolDirectionXAtBase = new double[3]; // 伺服运动模式工具X轴在Base坐标系中的方向
        protected double[] servoMotionToolDirectionYAtBase = new double[3]; // 伺服运动模式工具Y轴在Base坐标系中的方向
        protected double[] servoMotionToolDirectionZAtBase = new double[3]; // 伺服运动模式工具Z轴在Base坐标系中的方向
        protected double[] servoMotionBeginTcpPosition = new double[6]; //  伺服运动模式的初始Tcp位置

        protected double[] servoMotionPreservedForce = new double[6]; // 伺服运动模式要保持的力和力矩大小

        protected bool servoMotionActiveAbort = false; // 伺服运动模式是否被主动关闭
        protected bool servoMotionActivePause = false; // 伺服运动模式是否被主动暂停
        #endregion
        
        #region 属性
        /// <summary>
        /// 伺服运动模块标志属性
        /// </summary>
        protected virtual double ServoMotionFlag
        {
            get { return (double)ServoMotionModuleFlag.BaseMotion; }
        }

        /// <summary>
        /// 伺服运动模式是否打开
        /// </summary>
        public bool ServoMotionIfOpen
        {
            get { return servoMotionIfOpen; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        /// <param name="Port30004Used">是否使用30004端口</param>
        public ServoMotionBase(URDataProcessor Processor, bool Port30004Used)
        {
            internalProcessor = Processor;
            ifPort30004Used = Port30004Used;
        }

        /// <summary>
        /// 设定伺服运动主要参数
        /// </summary>
        /// <param name="ControlPeriod">伺服运动周期</param>
        /// <param name="LookAheadTime">伺服运动预计时间</param>
        /// <param name="Gain">伺服运动增益</param>
        protected void SetServoMotionParameters(double ControlPeriod = 0.008, double LookAheadTime = 0.1, double Gain = 200)
        {
            servoMotionControlPeriod = ControlPeriod;
            servoMotionLookAheadTime = LookAheadTime;
            servoMotionGain = Gain;
        }

        /// <summary>
        /// 伺服运动模式开始
        /// </summary>
        protected virtual void ServoMotionBegin()
        {
            // 初始化运动轮数并开始该模式
            servoMotionOpenRound = 0;
            servoMotionIfOpen = true;
        }

        /// <summary>
        /// 伺服运动模块执行的工作
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        public virtual void ServoMotionWork(double[] tcpRealPosition, double[] referenceForce)
        {
            // 1. 没有打开伺服运动模块即刻返回
            if (!servoMotionIfOpen)
            {
                return;
            }

            // 2. 进行运动前的准备工作
            if (servoMotionOpenRound < servoMotionInitialRound)
            {
                ServoMotionGetReady(tcpRealPosition, referenceForce);
                servoMotionOpenRound++;
                return;
            }

            // 3. 主动关闭模块或者达到终止条件即刻停止
            if (servoMotionActiveAbort || ServoMotionIfFinished(tcpRealPosition))
            {
                ServoMotionCompleted();
                return;
            }

            // 4. 计算伺服线运动量
            double[] nextTcpPosition = ServoMotionNextTcpPosition(tcpRealPosition, referenceForce);

            // 5. 发送计算得到的运动量
            ServoMotionPositionSend(nextTcpPosition);
        }

        /// <summary>
        /// 伺服运动模块的准备工作
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        protected abstract void ServoMotionGetReady(double[] tcpRealPosition, double[] referenceForce);

        /// <summary>
        /// 伺服运动模块是否到达终止条件
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <returns>返回是否终止</returns>
        protected virtual bool ServoMotionIfFinished(double[] tcpRealPosition)
        {
            return internalProcessor.ServoJugdeSingularReached();
        }

        /// <summary>
        /// 伺服运动模块停止工作
        /// </summary>
        protected virtual void ServoMotionCompleted()
        {
            // 停止运动 下位机停止
            internalProcessor.SendURCommanderStopL();

            // 重置部分参数 上位机停止
            internalProcessor.ServoSwitchMode(false);
            servoMotionActivePause = false;
            servoMotionActiveAbort = false;
            servoMotionIfOpen = false;
        }

        /// <summary>
        /// 伺服运动模块中计算下一周期的Tcp位置
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        /// <returns>返回下一周期的Tcp位置</returns>
        protected abstract double[] ServoMotionNextTcpPosition(double[] tcpRealPosition, double[] referenceForce);

        /// <summary>
        /// 伺服运动模块中的位置指令下达
        /// </summary>
        /// <param name="NextTcpCommand">下达的位置</param>
        protected virtual void ServoMotionPositionSend(double[] NextTcpCommand)
        {
            if (ifPort30004Used)
            {
                internalProcessor.SendURServorInputDatas(NextTcpCommand);
            }
            else
            {
                internalProcessor.SendURModbusInputDatas(NextTcpCommand);
            }
        }

        /// <summary>
        /// 主动关闭伺服运动模块
        /// </summary>
        public virtual void ServoMotionAbort()
        {
            if (servoMotionIfOpen)
            {
                servoMotionActiveAbort = true;
            }
        }

        /// <summary>
        /// 主动切换伺服运动模块启停状态
        /// </summary>      
        /// <param name="PauseOrNot">暂停与否</param>
        public virtual void ServoMotionSwitchPause(bool PauseOrNot)
        {
            if (servoMotionIfOpen)
            {
                servoMotionActivePause = PauseOrNot;
            }
        }

        /// <summary>
        /// 伺服运动模块部分配置参数输出
        /// </summary>
        /// <returns>配置参数输出</returns>
        protected abstract double[] ServoMotionOutputConfiguration();
        #endregion
    }
}
