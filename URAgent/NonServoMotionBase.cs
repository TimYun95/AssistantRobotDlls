using URCommunication;

namespace URNonServo
{
    /// <summary>
    /// 非伺服运动模块基类
    /// </summary>
    public abstract class NonServoMotionBase
    {
        #region 字段
        protected URDataProcessor internalProcessor; // 内部UR数据处理类，用以获得相关数据交换和控制权

        protected bool nonServoMotionIfOpen = false; // 非伺服运动模式是否打开
        protected int nonServoMotionOpenRound = 0; // 非伺服运动模式打开的周期轮数
        protected const int nonServoMotionInitialRound = 5; // 非伺服运动模式初始化周期轮数

        protected bool nonServoMotionActiveAbort = false; // 非伺服运动模式是否被主动关闭
        protected bool nonServoMotionActivePause = false; // 非伺服运动模式是否被主动暂停
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        public NonServoMotionBase(URDataProcessor Processor)
        {
            internalProcessor = Processor;
        }

        /// <summary>
        /// 非伺服运动模式开始
        /// </summary>
        protected virtual void NonServoMotionBegin()
        {
            // 初始化运动轮数并开始该模式
            nonServoMotionOpenRound = 0;
            nonServoMotionIfOpen = true;
        }

        /// <summary>
        /// 非伺服运动模块执行的工作
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        public virtual void NonServoMotionWork(double[] tcpRealPosition, double[] referenceForce)
        {
            // 1. 没有打开伺服运动模块即刻返回
            if (!nonServoMotionIfOpen)
            {
                return;
            }

            // 2. 进行运动前的准备工作
            if (nonServoMotionOpenRound < nonServoMotionInitialRound)
            {
                NonServoMotionGetReady(tcpRealPosition, referenceForce);
                nonServoMotionOpenRound++;
                return;
            }

            // 3. 主动关闭模块或者达到终止条件即刻停止
            if (nonServoMotionActiveAbort || NonServoMotionIfFinished(tcpRealPosition, referenceForce))
            {
                ServoMotionCompleted();
                return;
            }
        }

        /// <summary>
        /// 非伺服运动模块的准备工作
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        protected abstract void NonServoMotionGetReady(double[] tcpRealPosition, double[] referenceForce);

        /// <summary>
        /// 非伺服运动模块是否到达终止条件
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        /// <returns>返回是否终止</returns>
        protected abstract bool NonServoMotionIfFinished(double[] tcpRealPosition, double[] referenceForce);

        /// <summary>
        /// 伺服运动模块停止工作
        /// </summary>
        protected virtual void ServoMotionCompleted()
        {
            // 停止运动
            internalProcessor.SendURCommanderStopL();

            // 重置部分参数
            nonServoMotionActivePause = false;
            nonServoMotionActiveAbort = false;
            nonServoMotionIfOpen = false;
        }

        /// <summary>
        /// 主动关闭非伺服运动模块
        /// </summary>
        public virtual void NonServoMotionAbort()
        {
            if (nonServoMotionIfOpen)
            {
                nonServoMotionActiveAbort = true;
            }
        }

        /// <summary>
        /// 主动切换非伺服运动模块启停状态
        /// </summary>      
        /// <param name="PauseOrNot">暂停与否</param>
        public virtual void NonServoMotionSwitchPause(bool PauseOrNot)
        {
            if (nonServoMotionIfOpen)
            {
                nonServoMotionActivePause = PauseOrNot;
            }
        }
        #endregion
    }
}
