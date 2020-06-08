using System;
using System.Collections.Generic;

namespace MathFunction
{
    /// <summary>
    /// 四元数结构体
    /// </summary>
    public struct Quatnum
    {
        //四元数
        private double w;
        private double x;
        private double y;
        private double z;

        /// <summary>
        /// 输入四个元素的数组，分别对应w，x，y，z
        /// </summary>
        /// <param name="Data">输入的数组</param>
        public Quatnum(double[] Data)
        {
            w = Data[0];
            x = Data[1];
            y = Data[2];
            z = Data[3];
        }

        /// <summary>
        /// 获取四元数数据
        /// </summary>
        /// <returns>返回数组[w x y z]</returns>
        public double[] GetData()
        {
            return new double[] { this.w, this.x, this.y, this.z };
        }
    }

    /// <summary>
    /// 四元数数学运算基类，应保证参与运算的值均为单位值
    /// </summary>
    public class QuatnumMathBase
    {
        /// <summary>
        /// 四元数相乘，Q2 * Q1
        /// </summary>
        /// <param name="Q2">四元数q2</param>
        /// <param name="Q1">四元数q1</param>
        /// <returns>返回相乘结果</returns>
        public static Quatnum QuatnumMutiply(Quatnum Q2, Quatnum Q1)
        {
            double[] Q3 = new double[4];
            double[] q2Data = Q2.GetData();
            double[] q1Data = Q1.GetData();
            Q3[0] = q2Data[0] * q1Data[0] - q2Data[1] * q1Data[1] - q2Data[2] * q1Data[2] - q2Data[3] * q1Data[3];
            Q3[1] = q2Data[1] * q1Data[0] + q2Data[0] * q1Data[1] - q2Data[3] * q1Data[2] + q2Data[2] * q1Data[3];
            Q3[2] = q2Data[2] * q1Data[0] + q2Data[3] * q1Data[1] + q2Data[0] * q1Data[2] - q2Data[1] * q1Data[3];
            Q3[3] = q2Data[3] * q1Data[0] - q2Data[2] * q1Data[1] + q2Data[1] * q1Data[2] + q2Data[0] * q1Data[3];
            return new Quatnum(Q3);
        }

        /// <summary>
        /// 依次绕轴旋转的旋转矩阵用四元数表示，
        /// QMutiplyAxis[end] * ... * QMutiplyAxis[1] * QMutiplyAxis[0]
        /// </summary>
        /// <param name="QMutiplyAxis">四元数数组，
        /// 元素依次旋转，次序即排序</param>
        /// <returns>返回多次旋转叠加结果</returns>
        public static Quatnum QuatnumRotate(Quatnum[] QMutiplyAxis)
        {
            int rotateLength = QMutiplyAxis.Length;
            if (rotateLength > 2)
            {
                Quatnum[] QFrontMutiplyAxis = new Quatnum[rotateLength - 1];
                for (int k = 0; k < rotateLength - 1; k++)
                {
                    QFrontMutiplyAxis[k] = QMutiplyAxis[k];
                }
                return QuatnumMutiply(QMutiplyAxis[rotateLength - 1], QuatnumRotate(QFrontMutiplyAxis));
            }
            else if (rotateLength == 2)
            {
                return QuatnumMutiply(QMutiplyAxis[1], QMutiplyAxis[0]);
            }
            else
            {
                return new Quatnum(new double[] { 1.0, 0.0, 0.0, 0.0 });
            }
        }

        /// <summary>
        /// 四元数求逆
        /// </summary>
        /// <param name="Q">带求逆的四元数</param>
        /// <returns>返回求逆结果</returns>
        public static Quatnum InvQuatnum(Quatnum Q)
        {
            double[] QData = Q.GetData();
            return new Quatnum(new double[] { QData[0], -QData[1], -QData[2], -QData[3] });
        }

        /// <summary>
        /// 向量绕轴旋转一定角度后的结果用四元数表示，
        /// QRotateAxis * QBaseArray * QRotateAxis ^ (-1) 
        /// </summary>
        /// <param name="QRotateAxis">表示旋转的四元数</param>
        /// <param name="QBaseArray">待旋转的向量四元数表示</param>
        /// <returns>返回旋转后得到的向量四元数表示</returns>
        public static Quatnum RotateAlongAxis(Quatnum QRotateAxis, Quatnum QBaseArray)
        {
            return QuatnumMutiply(QuatnumMutiply(QRotateAxis, QBaseArray), InvQuatnum(QRotateAxis));
        }

        /// <summary>
        /// 向量的四元数表示转化为向量的一般表示
        /// </summary>
        /// <param name="Q">输入向量的四元数表示</param>
        /// <returns>返回向量的一般表示</returns>
        public static double[] Quatnum2Array(Quatnum Q)
        {
            double[] QData = Q.GetData();
            return new double[] { QData[1], QData[2], QData[3] };
        }

        /// <summary>
        /// 旋转的四元数表示转换为轴角表示
        /// </summary>
        /// <param name="Q">输入旋转的四元数表示</param>
        /// <returns>返回旋转的轴角表示</returns>
        public static double[] Quatnum2AxisAngle(Quatnum Q)
        {
            double[] QData = Q.GetData();
            if (Math.Abs(QData[0]) > 1.0) // 限幅到-1到+1之内
            {
                QData[0] = (Math.Sign(QData[0]) == 1) ? 1.0 : -1.0;
            }

            if (QData[0] == 1 || QData[0] == -1) // 旋转整周，相当于不转
            {
                return new double[] { 0.0, 0.0, 0.0 };
            }
            else
            {
                double angle = Math.Acos(QData[0]); // 返回在0到pi之间
                double[] axis = { QData[1] / Math.Sin(angle), QData[2] / Math.Sin(angle), QData[3] / Math.Sin(angle) };
                angle *= 2.0; // 返回在0到2pi之间

                if (angle >= Math.PI)
                {
                    angle = -(2.0 * Math.PI - angle);
                } // 返回在-pi到pi之间

                return new double[] { axis[0] * angle, axis[1] * angle, axis[2] * angle };
            }
        }

        /// <summary>
        /// 向量的一般表示转化为向量的四元数表示
        /// </summary>
        /// <param name="NormalArray">输入向量的一般表示</param>
        /// <returns>返回向量的四元数表示</returns>
        public static Quatnum Array2Quatnum(double[] NormalArray)
        {
            return new Quatnum(new double[] { 0.0, NormalArray[0], NormalArray[1], NormalArray[2] });
        }

        /// <summary>
        /// 旋转的轴角表示转换为四元数表示
        /// </summary>
        /// <param name="AxisAngle">输入旋转的轴角表示</param>
        /// <returns>返回四元数</returns>
        public static Quatnum AxisAngle2Quatnum(double[] AxisAngle)
        {
            if (AxisAngle[0] == 0.0 && AxisAngle[1] == 0.0 && AxisAngle[2] == 0.0) // 相当于不转
            {
                return new Quatnum(new double[] { 1.0, 0.0, 0.0, 0.0 });
            }
            else
            {
                double angle = Math.Sqrt(Math.Pow(AxisAngle[0], 2) + Math.Pow(AxisAngle[1], 2) + Math.Pow(AxisAngle[2], 2));
                double[] axis = { AxisAngle[0] / angle, AxisAngle[1] / angle, AxisAngle[2] / angle };
                return new Quatnum(new double[] { Math.Cos(angle / 2), Math.Sin(angle / 2) * axis[0], Math.Sin(angle / 2) * axis[1], Math.Sin(angle / 2) * axis[2] });
            }
        }

        /// <summary>
        /// 自定义列主元法求解4阶线性方程组
        /// </summary>
        /// <param name="Ao">系数矩阵</param>
        /// <param name="bo">列空间投影</param>
        /// <returns>返回解向量</returns>
        public static double[] SolveEqu4Self(double[,] Ao, double[] bo)
        {
            double[,] A = new double[4, 4];
            double[] b = new double[4];
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    A[i, j] = Ao[i, j];
                }
                b[i] = bo[i];
            }
            int[] seq = new int[4] { 0, 0, 0, 0 };

            for (int k = 1; k < 4; k++)
            {
                if (Math.Abs(A[k, 0]) > Math.Abs(A[seq[0], 0]))
                {
                    seq[0] = k;
                }
            }
            b[seq[0]] = b[seq[0]] / A[seq[0], 0];
            for (int k = 3; k >= 0; k--)
            {
                A[seq[0], k] = A[seq[0], k] / A[seq[0], 0];
            }
            for (int i = 0; i < 4; i++)
            {
                if (i != seq[0])
                {
                    b[i] -= (b[seq[0]] * A[i, 0]);
                    for (int k = 3; k >= 0; k--)
                    {
                        A[i, k] -= (A[seq[0], k] * A[i, 0]);
                    }
                }
            }

            if (seq[0] == 0)
            {
                seq[1] = 1;
            }
            for (int k = 1; k < 4; k++)
            {
                if (k != seq[0] && Math.Abs(A[k, 1]) > Math.Abs(A[seq[1], 1]))
                {
                    seq[1] = k;
                }
            }
            b[seq[1]] = b[seq[1]] / A[seq[1], 1];
            for (int k = 3; k > 0; k--)
            {
                A[seq[1], k] = A[seq[1], k] / A[seq[1], 1];
            }
            for (int i = 0; i < 4; i++)
            {
                if (i != seq[1])
                {
                    b[i] -= (b[seq[1]] * A[i, 1]);
                    for (int k = 3; k > 0; k--)
                    {
                        A[i, k] -= (A[seq[1], k] * A[i, 1]);
                    }
                }
            }

            int[] kt = { 0, 0 }; int nt = 0;
            for (int k = 0; k < 4; k++)
            {
                if (k != seq[0] && k != seq[1])
                {
                    kt[nt] = k;
                    nt++;
                }
            }
            if (Math.Abs(A[kt[0], 2]) > Math.Abs(A[kt[1], 2]))
            {
                seq[2] = kt[0];
                seq[3] = kt[1];
            }
            else
            {
                seq[2] = kt[1];
                seq[3] = kt[0];
            }
            b[seq[2]] = b[seq[2]] / A[seq[2], 2];
            for (int k = 3; k > 0; k--)
            {
                A[seq[2], k] = A[seq[2], k] / A[seq[2], 2];
            }
            for (int i = 0; i < 4; i++)
            {
                if (i != seq[2])
                {
                    b[i] -= (b[seq[2]] * A[i, 2]);
                    for (int k = 3; k > 0; k--)
                    {
                        A[i, k] -= (A[seq[2], k] * A[i, 2]);
                    }
                }
            }

            b[seq[3]] = b[seq[3]] / A[seq[3], 3];
            A[seq[3], 3] = 1;
            for (int i = 0; i < 4; i++)
            {
                if (i != seq[3])
                {
                    b[i] -= (b[seq[3]] * A[i, 3]);
                    A[i, 3] = 0;
                }
            }

            return new double[] { b[seq[0]], b[seq[1]], b[seq[2]], b[seq[3]] };
        }

        /// <summary>
        /// 寻找两个四元数之间的过渡四元数，即
        /// Q2 = Qt * Q1，已知Q1，Q2，求Qt
        /// </summary>
        /// <param name="Q1">初始四元数</param>
        /// <param name="Q2">终态四元数</param>
        /// <returns>返回过渡四元数</returns>
        public static Quatnum FindTransitQuatnum(Quatnum Q1, Quatnum Q2)
        {
            double[] Q1Data = Q1.GetData();
            double[] Q2Data = Q2.GetData();

            double[,] A = {{Q1Data[0], -Q1Data[1], -Q1Data[2], -Q1Data[3]},
                                     {Q1Data[1], Q1Data[0], Q1Data[3], -Q1Data[2]},
                                     {Q1Data[2], -Q1Data[3], Q1Data[0], Q1Data[1]},
                                     {Q1Data[3], Q1Data[2], -Q1Data[1], Q1Data[0]}};

            return new Quatnum(SolveEqu4Self(A, Q2Data));
        }
    }
}
