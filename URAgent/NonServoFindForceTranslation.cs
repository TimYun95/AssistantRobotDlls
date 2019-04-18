using System;
using URCommunication;
using MathFunction;

namespace URNonServo
{
    /// <summary>
    /// 非伺服寻力运动模块
    /// </summary>
    public class NonServoFindForceTranslation : NonServoMotionBase
    {
        #region 枚举
        /// <summary>
        /// Tcp坐标系中的运动方向
        /// </summary>
        public enum NonServoDirectionAtTcp : byte
        {
            PositiveX = 0,
            NegativeX,
            PositiveY,
            NegativeY,
            PositiveZ,
            NegativeZ
        }
        #endregion

        #region 字段
        protected bool nonServoAutoMovingDirectionJudge = false; // 运动方向的自动判断
        protected bool nonServoMovingDirectionReverse = false; // 运动方向是否取反

        protected NonServoDirectionAtTcp nonServoMovingDirectionAtTcp = NonServoDirectionAtTcp.PositiveZ; // 相对Tcp坐标系的运动方向
        protected double[] nonServoMovingDirectionArrayAtTcp = new double[3]; // 相对Tcp坐标系的运动方向
        protected double[] nonServoMovingDirectionArrayAtBase = new double[3]; // 相对Base坐标系的运动方向

        protected double nonServoMotionMovingSpeed = 0.0; // 寻力方向的移动速度
        protected double nonServoMotionMovingAcceleration = 0.0; // 寻力方向的移动加速度
        protected double nonServoMotionDetectingForce = 0.0; // 寻力方向上到达停止条件的力绝对值
        protected double[] nonServoMotionMovingArray = new double[6]; // 寻力方向移动指令数组

        protected double nonServoMotionEndForce = 0.0; // 寻力方向上停止时的力大小
        #endregion

        #region 属性
        /// <summary>
        /// 寻力方向上停止时的力大小
        /// </summary>
        public double NonServoMotionEndForce
        {
            get { return nonServoMotionEndForce; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="Processor">UR数据处理器引用</param>
        public NonServoFindForceTranslation(URDataProcessor Processor)
            : base(Processor) { }

        /// <summary>
        /// 非伺服运动模块设置并开始
        /// </summary>
        /// <param name="MoveDirectionAtTcp">相对Tcp坐标系的运动方向</param>
        /// <param name="tcpCurrentPosition">实时Tcp坐标</param>
        /// <param name="Velocity">寻力方向运动速度</param>
        /// <param name="Acceleration">寻力方向运动加速度</param>
        /// <param name="DetectForce">寻力方向终止力大小</param>
        /// <param name="IfAutoFindDirectionInDetail">是否自动寻找运动方向正负</param>
        public void NonServoMotionSetAndBegin(NonServoDirectionAtTcp MoveDirectionAtTcp,
                                                                           double[] tcpCurrentPosition,
                                                                           double Velocity,
                                                                           double Acceleration,
                                                                           double DetectForce,
                                                                           bool IfAutoFindDirectionInDetail = false)
        {
            // 设置是否自动寻找力正负方向
            nonServoAutoMovingDirectionJudge = IfAutoFindDirectionInDetail;

            // 设置寻力方向
            nonServoMovingDirectionAtTcp = MoveDirectionAtTcp;

            // 获得寻力方向在Base系中的表示
            double[] tcpToBase = URMath.ReverseReferenceRelationship(tcpCurrentPosition);
            Quatnum qTcpToBase = URMath.AxisAngle2Quatnum(new double[] { tcpToBase[3], tcpToBase[4], tcpToBase[5] });
            switch (nonServoMovingDirectionAtTcp)
            {
                case NonServoDirectionAtTcp.PositiveX:
                    nonServoMovingDirectionArrayAtTcp = new double[] { 1.0, 0.0, 0.0 };
                    break;
                case NonServoDirectionAtTcp.NegativeX:
                    nonServoMovingDirectionArrayAtTcp = new double[] { -1.0, 0.0, 0.0 };
                    break;
                case NonServoDirectionAtTcp.PositiveY:
                    nonServoMovingDirectionArrayAtTcp = new double[] { 0.0, 1.0, 0.0 };
                    break;
                case NonServoDirectionAtTcp.NegativeY:
                    nonServoMovingDirectionArrayAtTcp = new double[] { 0.0, -1.0, 0.0 };
                    break;
                case NonServoDirectionAtTcp.PositiveZ:
                    nonServoMovingDirectionArrayAtTcp = new double[] { 0.0, 0.0, 1.0 };
                    break;
                case NonServoDirectionAtTcp.NegativeZ:
                    nonServoMovingDirectionArrayAtTcp = new double[] { 0.0, 0.0, -1.0 };
                    break;
                default:
                    break;
            }
            nonServoMovingDirectionArrayAtBase = URMath.FindDirectionToSecondReferenceFromFirstReference(nonServoMovingDirectionArrayAtTcp, qTcpToBase);

            // 设置寻力运动参数
            nonServoMotionMovingSpeed = Velocity;
            nonServoMotionMovingAcceleration = Acceleration;

            // 设置终止力大小
            nonServoMotionDetectingForce = DetectForce;

            // 开启本非伺服模式
            NonServoMotionBegin();
        }

        /// <summary>
        /// 非伺服运动的准备工作
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        protected override void NonServoMotionGetReady(double[] tcpRealPosition, double[] referenceForce)
        {
            switch (nonServoMotionOpenRound)
            {
                case 0:
                    nonServoMovingDirectionReverse = false;
                    if (nonServoAutoMovingDirectionJudge)
                    {
                        double forceMagnitude = URMath.VectorDotMultiply(referenceForce, nonServoMovingDirectionArrayAtTcp);
                        if (Math.Abs(forceMagnitude) >= nonServoMotionDetectingForce) nonServoMovingDirectionReverse = true;
                    }
                    break;
                case 1:
                case 2:
                    if (nonServoAutoMovingDirectionJudge && nonServoMovingDirectionReverse)
                    {
                        double forceMagnitude = URMath.VectorDotMultiply(referenceForce, nonServoMovingDirectionArrayAtTcp);
                        if (Math.Abs(forceMagnitude) >= nonServoMotionDetectingForce) nonServoMovingDirectionReverse = true;
                        else nonServoMovingDirectionReverse = false;
                    }
                    break;
                case 3: // 第四个周期 更新移动指令数组
                    if (nonServoMovingDirectionReverse)
                    {
                        nonServoMotionMovingArray[0] = -nonServoMovingDirectionArrayAtBase[0] * nonServoMotionMovingSpeed;
                        nonServoMotionMovingArray[1] = -nonServoMovingDirectionArrayAtBase[1] * nonServoMotionMovingSpeed;
                        nonServoMotionMovingArray[2] = -nonServoMovingDirectionArrayAtBase[2] * nonServoMotionMovingSpeed;
                    }
                    else
                    {
                        nonServoMotionMovingArray[0] = nonServoMovingDirectionArrayAtBase[0] * nonServoMotionMovingSpeed;
                        nonServoMotionMovingArray[1] = nonServoMovingDirectionArrayAtBase[1] * nonServoMotionMovingSpeed;
                        nonServoMotionMovingArray[2] = nonServoMovingDirectionArrayAtBase[2] * nonServoMotionMovingSpeed;
                    }
                    nonServoMotionMovingArray[3] = 0.0;
                    nonServoMotionMovingArray[4] = 0.0;
                    nonServoMotionMovingArray[5] = 0.0;
                    break;
                case 4: // 第五个周期 下达下位机指令并运行
                    internalProcessor.SendURCommanderSpeedL(nonServoMotionMovingArray, nonServoMotionMovingAcceleration, 3600);
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// 非伺服运动模块是否到达终止条件
        /// </summary>
        /// <param name="tcpRealPosition">实时Tcp坐标</param>
        /// <param name="referenceForce">参考力信号</param>
        /// <returns>返回是否终止</returns>
        protected override bool NonServoMotionIfFinished(double[] tcpRealPosition, double[] referenceForce)
        {
            // 寻力方向力大小终止
            double forceMagnitude = URMath.VectorDotMultiply(referenceForce, nonServoMovingDirectionArrayAtTcp);
            if (nonServoMovingDirectionReverse)
            {
                if (Math.Abs(forceMagnitude) <= nonServoMotionDetectingForce)
                {
                    nonServoMotionEndForce = forceMagnitude;
                    return true;
                }
            }
            else
            {
                if (Math.Abs(forceMagnitude) >= nonServoMotionDetectingForce)
                {
                    nonServoMotionEndForce = forceMagnitude;
                    return true;
                }
            }
            return false;
        }
        #endregion
    }
}
