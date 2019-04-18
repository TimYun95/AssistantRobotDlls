using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Reflection;
using URCommunication;
using XMLConnection;
using LogPrinter;

namespace URModule
{
    /// <summary>
    /// 执行模块基类
    /// </summary>
    public abstract class OperateModuleBase
    {
        #region 枚举
        /// <summary>
        /// 工作状态字
        /// </summary>
        public enum WorkStatus : short
        {
            NotAtWork = -1,
            CanDoWork = 0,
            InitialForceDevice = 1,
            ParametersConfiguration = 2,
            ReadyToWork = 3,
            WorkRunning = 4
        }
        #endregion

        #region 字段
        protected URDataProcessor internalProcessor; // 内部UR数据处理类，用以获得相关数据交换和控制权
        protected XMLConnector xmlProcessor; // XML文件处理者

        protected bool ifAutoReplaceConfiguration = true; // XML文件自动转存开关
        protected bool ifConfirmParametersOnce = false; // 是否确认过所有的配置参数
        protected bool ifEndModuleRunningImmediately = false; // 是否立即停止模块运动

        protected WorkStatus workingStatus = WorkStatus.NotAtWork; // 工作状态标志字
        private static readonly object lockedVariable = new object(); // 线程锁变量 锁工作状态标志读写

        protected double[] initialJointAngles = new double[6]; // 初始关节角度
        protected double[] installTcpPosition = new double[6]; // 安装工具Tcp位置
        protected bool installHanged = false; // 是否倒装
        protected double toolMass = 0.0; // 工具重力

        protected bool forceSensorCleared = false; // 力传感器是否清零
        protected bool ifEverStopImmediately = false; // 是否执行过立即停止

        protected const double normalMoveAccelerationL = 0.1; // 普通移动加速度
        protected const double normalMoveSpeedL = 0.1; // 普通移动速度
        protected const double fastMoveAccelerationL = 0.2; // 快速移动加速度
        protected const double fastMoveSpeedL = 0.2; // 快速移动速度

        public delegate void SendShort(short Status); // short类型发送委托
        /// <summary>
        /// 发送当前程序状态
        /// </summary>
        public event SendShort OnSendWorkingStatus;

        public delegate void SendBool(bool Status); // bool类型发送委托
        /// <summary>
        /// 发送当前参数确认情况
        /// </summary>
        public event SendBool OnSendConfirmParametersStatus;
        /// <summary>
        /// 发送当前力传感器清零状态
        /// </summary>
        public event SendBool OnSendForceClearedStatus;
        #endregion

        #region 属性
        /// <summary>
        /// 数据库中记录的当前工具的初始关节角度
        /// </summary>
        public double[] InitialJointAngles
        {
            set
            {
                initialJointAngles = (double[])value.Clone();
            }
        }

        /// <summary>
        /// 数据库中记录的当前工具的安装Tcp位置
        /// </summary>
        public double[] InstallTcpPosition
        {
            set
            {
                installTcpPosition = (double[])value.Clone();
            }
        }

        /// <summary>
        /// 数据库中记录的当前工具的安装方式
        /// </summary>
        public bool InstallHanged
        {
            set
            {
                installHanged = value;
            }
        }

        /// <summary>
        /// 数据库中记录的当前工具的重力
        /// </summary>
        public double ToolMass
        {
            set
            {
                toolMass = value;
            }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="InternalProcessor">内部UR数据处理类，用以获得相关数据交换和控制权</param>
        /// <param name="RecordedJointAngles">数据库中记录的当前工具的初始关节角度</param>
        /// <param name="InstallTcpPosition">数据库中记录的当前工具的安装TCP位置</param>
        /// <param name="InstallHanged">数据库中记录的当前工具的安装方式</param>
        /// <param name="ToolMass">数据库中记录的当前工具的重力</param>
        public OperateModuleBase(URDataProcessor InternalProcessor, double[] RecordedJointAngles, double[] InstallTcpPosition, bool InstallHanged, double ToolMass)
        {
            internalProcessor = InternalProcessor;
            initialJointAngles = (double[])RecordedJointAngles.Clone();
            installTcpPosition = (double[])InstallTcpPosition.Clone();
            installHanged = InstallHanged;
            toolMass = ToolMass;
            internalProcessor.OnSendZeroedForceCompeleted += ForceSenorClearingFinished;
        }

        /// <summary>
        /// 从string列表获得参数并保存到XML文件中
        /// </summary>
        /// <param name="ParameterStringList">参数string列表</param>
        public void SaveParametersFromStringToXml(List<string> ParameterStringList)
        {
            GetParametersFromString(ParameterStringList);
            SaveParametersToXml();
        }

        /// <summary>
        /// 从string列表中获得并保存到模块参数
        /// </summary>
        /// <param name="ParameterStringList">参数string列表</param>
        protected abstract void GetParametersFromString(List<string> ParameterStringList);

        /// <summary>
        /// 将模块参数保存到XML文件中
        /// </summary>
        protected abstract void SaveParametersToXml();

        /// <summary>
        /// 从XML文件中加载到模块参数并输出
        /// </summary>
        public void LoadParametersFromXmlAndOutput()
        {
            LoadParametersFromXml();
            OutputParameters();
        }

        /// <summary>
        /// 从XML文件中加载到模块参数
        /// </summary>
        protected abstract void LoadParametersFromXml();

        /// <summary>
        /// 将模块参数抛出
        /// </summary>
        protected abstract void OutputParameters();

        /// <summary>
        /// 初始化XML文件处理者
        /// </summary>
        protected abstract void InitialXmlProcessor();

        /// <summary>
        /// 自动转存XML文件
        /// </summary>
        protected void AutoReplaceXml()
        {
            xmlProcessor.ReplaceXml(DateTime.Now.ToString("yyyy-MM-dd") + " " + DateTime.Now.Hour.ToString() + "-" + DateTime.Now.Minute.ToString() + "-" + DateTime.Now.Second.ToString() + ".xml");
        }

        /// <summary>
        /// 激活本模块
        /// </summary>
        public void ActiveModule()
        {
            lock (lockedVariable)
            {
                if (workingStatus == WorkStatus.NotAtWork)
                {
                    workingStatus = WorkStatus.CanDoWork;
                    OnSendWorkingStatus((short)workingStatus);
                }
                else return;
            }

            AttemptToStartModule();
        }

        /// <summary>
        /// 准备开始运行模块
        /// </summary>
        protected virtual void AttemptToStartModule()
        {
            ifConfirmParametersOnce = false;
            OnSendConfirmParametersStatus(ifConfirmParametersOnce);
            ifEndModuleRunningImmediately = false;
            ifEverStopImmediately = false;
        }

        /// <summary>
        /// 冻结本模块
        /// </summary>
        public void FreezeModule()
        {
            lock (lockedVariable)
            {
                if (workingStatus == WorkStatus.CanDoWork)
                {
                    workingStatus = WorkStatus.NotAtWork;
                    OnSendWorkingStatus((short)workingStatus);
                }
            }
        }

        /// <summary>
        /// 初始化力传感器
        /// </summary>
        public virtual void InitialForceSensor()
        {
            // 判断是否可以运动
            if (internalProcessor.IfNearSingularPoint || !internalProcessor.IfURConnected) return;

            lock (lockedVariable)
            {
                if (!ifEverStopImmediately && workingStatus == WorkStatus.CanDoWork)
                {
                    forceSensorCleared = false;
                    OnSendForceClearedStatus(forceSensorCleared);
                    workingStatus = WorkStatus.InitialForceDevice;
                    OnSendWorkingStatus((short)workingStatus);
                }
                else return;
            }

            Task.Run(new Action(() =>
            {
                // 1. 移动到数据库记录的初始位置
                internalProcessor.SendURCommanderMoveLViaJ(initialJointAngles, normalMoveAccelerationL, normalMoveSpeedL);
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Clear the force sensor, first go to initial joint position.");

                Thread.Sleep(800);
                if (!JudgeIfMotionCanBeContinued()) return;
                while (internalProcessor.ProgramState == (double)URDataProcessor.RobotProgramStatus.Running)
                {
                    Thread.Sleep(200);
                    if (!JudgeIfMotionCanBeContinued()) return;
                }
                if (!JudgeIfMotionCanBeContinued()) return;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Clear the force sensor, already arrive at initial joint position.");

                // 等待机械臂稳定
                Thread.Sleep(1000);
                if (!JudgeIfMotionCanBeContinued()) return;
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Clear the force sensor, robot has been stable.");

                // 重置力传感器
                internalProcessor.SetOPTOBias(true);
                Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Clearing the force sensor.");
            }));
        }

        /// <summary>
        /// 力传感器清零结束
        /// </summary>
        protected void ForceSenorClearingFinished()
        {
            forceSensorCleared = true;
            OnSendForceClearedStatus(forceSensorCleared);
        }

        /// <summary>
        /// 进入参数配置状态
        /// </summary>
        public void EnterParameterConfiguration()
        {
            lock (lockedVariable)
            {
                if (forceSensorCleared && workingStatus == WorkStatus.InitialForceDevice)
                {
                    ifConfirmParametersOnce = false;
                    OnSendConfirmParametersStatus(ifConfirmParametersOnce);
                    workingStatus = WorkStatus.ParametersConfiguration;
                    OnSendWorkingStatus((short)workingStatus);
                }
            }
        }

        /// <summary>
        /// 确认配置参数设置并保存
        /// </summary>
        /// <param name="ParameterStringList"></param>
        public virtual void ConfirmConfigurationParameters(List<string> ParameterStringList)
        {
            lock (lockedVariable)
            {
                if (workingStatus == WorkStatus.ParametersConfiguration)
                {
                    ifConfirmParametersOnce = true;
                    OnSendConfirmParametersStatus(ifConfirmParametersOnce);
                    SaveParametersFromStringToXml(ParameterStringList);
                }
                else return;
            }
        }

        /// <summary>
        /// 准备开始执行任务
        /// </summary>
        public void BeReadyToWork()
        {
            lock (lockedVariable)
            {
                if (ifConfirmParametersOnce && ((forceSensorCleared && workingStatus == WorkStatus.InitialForceDevice) || workingStatus == WorkStatus.ParametersConfiguration))
                {
                    workingStatus = WorkStatus.ReadyToWork;
                    OnSendWorkingStatus((short)workingStatus);
                }
            }
        }

        /// <summary>
        /// 模块任务开始运行
        /// </summary>
        public void StartModuleNow()
        {
            lock (lockedVariable)
            {
                if (workingStatus == WorkStatus.ReadyToWork)
                {
                    workingStatus = WorkStatus.WorkRunning;
                    OnSendWorkingStatus((short)workingStatus);
                }
                else return;
            }

            ModuleWork();
        }

        /// <summary>
        /// 模块执行的工作
        /// </summary>
        protected abstract void ModuleWork();

        /// <summary>
        /// 模块任务运行结束
        /// </summary>
        protected void StopModuleNow()
        {
            lock (lockedVariable)
            {
                if (workingStatus == WorkStatus.WorkRunning)
                {
                    workingStatus = WorkStatus.CanDoWork;
                    OnSendWorkingStatus((short)workingStatus);
                    StopModuleWork();
                }
                else return;
            }
        }

        /// <summary>
        /// 停止模块执行工作中的运动
        /// </summary>
        protected abstract void StopModuleWork();

        /// <summary>
        /// 立刻停止模块运行
        /// </summary>
        public void EndModuleNow()
        {
            lock (lockedVariable)
            {
                if (workingStatus != WorkStatus.NotAtWork)
                {
                    workingStatus = WorkStatus.CanDoWork;
                    OnSendWorkingStatus((short)workingStatus);
                    StopAllMotionInModule();
                }
                else return;
            }
        }

        /// <summary>
        /// 停止所有模块涉及的运动
        /// </summary>
        protected virtual void StopAllMotionInModule()
        {
            ifEndModuleRunningImmediately = true;
            ifEverStopImmediately = true;
            
            StopRelevantModule();
        }

        /// <summary>
        /// 停止所涉及的模块
        /// </summary>
        protected abstract void StopRelevantModule();

        /// <summary>
        /// 从立即停止中恢复为正常状态
        /// </summary>
        public virtual void RecoverToNormal()
        {
            lock (lockedVariable)
            {
                if (ifEverStopImmediately && workingStatus == WorkStatus.CanDoWork)
                {
                    ifEverStopImmediately = false;
                    ifEndModuleRunningImmediately = false;
                }
                else return;
            }
        }

        /// <summary>
        /// 判断是否可以继续执行运动
        /// </summary>
        /// <returns>返回判断结果</returns>
        protected bool JudgeIfMotionCanBeContinued()
        {
            if (!internalProcessor.IfURConnected) // 网络连接中断则结束
            {
                return false;
            }
            if (ifEndModuleRunningImmediately) // 要求停止运行则结束
            {
                ifEndModuleRunningImmediately = false;
                return false;
            }
            return true;
        }

        /// <summary>
        /// 计算并检查所有相关参数
        /// </summary>
        protected abstract void CalculateAndCheckParametersBothExposedAndHidden();
        #endregion
    }
}
