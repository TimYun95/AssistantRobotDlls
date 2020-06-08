using System;
using System.Collections.Generic;

namespace MathFunction
{
    /// <summary>
    /// UR中有关运算的类
    /// </summary>
    public class URMath : QuatnumMathBase
    {
        /// <summary>
        /// 计算向量模大小
        /// </summary>
        /// <param name="GivenArray">待算向量</param>
        /// <returns>返回向量模大小</returns>
        public static double LengthOfArray(double[] GivenArray)
        {
            return Math.Sqrt(Math.Pow(GivenArray[0], 2) + Math.Pow(GivenArray[1], 2) + Math.Pow(GivenArray[2], 2));
        }

        /// <summary>
        /// 将向量的方向在第一个坐标系中的表示转换到在第二个坐标系中的表示
        /// </summary>
        /// <param name="DirectionToFirstReference">在第一个坐标系中的向量方向表示</param>
        /// <param name="PostureFromFirstToSecondReference">第一个坐标系到第二个坐标系的姿态转换关系</param>
        /// <returns>返回在第二个坐标系中的向量方向表示</returns>
        public static double[] FindDirectionToSecondReferenceFromFirstReference(double[] DirectionToFirstReference, Quatnum PostureFromFirstToSecondReference)
        {
            Quatnum qDirection1 = Array2Quatnum(DirectionToFirstReference);
            Quatnum qDirection2 = RotateAlongAxis(InvQuatnum(PostureFromFirstToSecondReference), qDirection1);
            double[] directionToSecondReference = Quatnum2Array(qDirection2);

            return directionToSecondReference;
        }

        /// <summary>
        /// 将力或者位姿在第一个坐标系中的表示转换到在第二个坐标系中的表示
        /// 如果被转换对象是力，则输入力的格式为[力大小(1) 力方向(3) 力矩大小(1) 力矩方向(3)]
        /// 如果被转换对象是位姿，则输入位姿的格式为[位置(3) 姿态轴角表示(3)]
        /// 小括号内的数字表示该变量的维度，即力为8维向量，位姿为6维向量
        /// </summary>
        /// <param name="CordinateToFirstReference">在第一个坐标系中的坐标表示</param>
        /// <param name="TransformFromFirstToSecondReference">第一个坐标系到第二个坐标系的转换关系</param>
        /// <returns>返回在第二个坐标系中的坐标表示</returns>
        public static double[] FindCordinateToSecondReferenceFromFirstReference(double[] CordinateToFirstReference, double[] TransformFromFirstToSecondReference)
        {
            int cordinateLength = CordinateToFirstReference.Length;
            if (cordinateLength == 8) // 力信号转换
            {
                Quatnum qPosture1To2 = AxisAngle2Quatnum(new double[] { TransformFromFirstToSecondReference[3], TransformFromFirstToSecondReference[4], TransformFromFirstToSecondReference[5] });

                double forceAmplitude = CordinateToFirstReference[0];
                double[] forceDirection1 = new double[] { CordinateToFirstReference[1], CordinateToFirstReference[2], CordinateToFirstReference[3] };
                double[] forceDirection2 = FindDirectionToSecondReferenceFromFirstReference(forceDirection1, qPosture1To2);

                double torqueAmplitude = CordinateToFirstReference[4];
                double[] torqueDirection1 = new double[] { CordinateToFirstReference[5], CordinateToFirstReference[6], CordinateToFirstReference[7] };
                double[] torqueDirection2 = FindDirectionToSecondReferenceFromFirstReference(torqueDirection1, qPosture1To2);

                double[] cordinateToSecondReference = { forceAmplitude * forceDirection2[0], forceAmplitude * forceDirection2[1], forceAmplitude * forceDirection2[2],
                                                                                   torqueAmplitude * torqueDirection2[0], torqueAmplitude * torqueDirection2[1], torqueAmplitude * torqueDirection2[2] };

                return cordinateToSecondReference;
            }
            else if (cordinateLength == 6) // 坐标信号转换
            {
                double[] position1To2 = { TransformFromFirstToSecondReference[0], TransformFromFirstToSecondReference[1], TransformFromFirstToSecondReference[2] };
                Quatnum qPosture1To2 = AxisAngle2Quatnum(new double[] { TransformFromFirstToSecondReference[3], TransformFromFirstToSecondReference[4], TransformFromFirstToSecondReference[5] });

                double[] position1 = { CordinateToFirstReference[0], CordinateToFirstReference[1], CordinateToFirstReference[2] };
                double positionLength = LengthOfArray(position1);
                double[] positionDirection1;
                if (positionLength == 0)
                {
                    positionDirection1 = new double[3] { 0.0, 0.0, 0.0 };
                }
                else
                {
                    positionDirection1 = new double[3] { position1[0] / positionLength, position1[1] / positionLength, position1[2] / positionLength };
                }
                double[] positionDirection2 = FindDirectionToSecondReferenceFromFirstReference(positionDirection1, qPosture1To2);
                double position1To2Length = LengthOfArray(position1To2);
                double[] position1To2Direction1;
                if (position1To2Length == 0)
                {
                    position1To2Direction1 = new double[3] { 0.0, 0.0, 0.0 };

                }
                else
                {
                    position1To2Direction1 = new double[3] { position1To2[0] / position1To2Length, position1To2[1] / position1To2Length, position1To2[2] / position1To2Length };
                }
                double[] position1To2Direction2 = FindDirectionToSecondReferenceFromFirstReference(position1To2Direction1, qPosture1To2);
                double[] position2 = { -position1To2Direction2[0] * position1To2Length + positionDirection2[0] * positionLength, -position1To2Direction2[1] * position1To2Length + positionDirection2[1] * positionLength, -position1To2Direction2[2] * position1To2Length + positionDirection2[2] * positionLength };

                double[] posture1 = { CordinateToFirstReference[3], CordinateToFirstReference[4], CordinateToFirstReference[5] };
                double posture1Angle = LengthOfArray(posture1);
                double[] posture1Direction;
                if (posture1Angle == 0)
                {
                    posture1Direction = new double[3] { 0.0, 0.0, 0.0 };
                }
                else
                {
                    posture1Direction = new double[3] { posture1[0] / posture1Angle, posture1[1] / posture1Angle, posture1[2] / posture1Angle };
                }
                double[] posture1Direction2 = FindDirectionToSecondReferenceFromFirstReference(posture1Direction, qPosture1To2);
                double[] posture12 = { posture1Direction2[0] * posture1Angle, posture1Direction2[1] * posture1Angle, posture1Direction2[2] * posture1Angle };
                Quatnum qPosture12 = AxisAngle2Quatnum(posture12);

                Quatnum qPosture2 = QuatnumRotate(new Quatnum[] { InvQuatnum(qPosture1To2), qPosture12 });
                double[] posture2 = Quatnum2AxisAngle(qPosture2);

                double[] cordinateToSecondReference = { position2[0], position2[1], position2[2],
                                                                                   posture2[0], posture2[1], posture2[2] };

                return cordinateToSecondReference;
            }
            else // 转换输入出错
            {
                return null;
            }
        }

        /// <summary>
        /// 已知第一个坐标系到第二个坐标系的转换，获得第二个坐标系到第一个坐标系的转换
        /// </summary>
        /// <param name="Relationship">第一个坐标系到第二个坐标系的转换</param>
        /// <returns>返回第二个坐标系到第一个坐标系的转换</returns>
        public static double[] ReverseReferenceRelationship(double[] Relationship)
        {
            double[] position = { Relationship[0], Relationship[1], Relationship[2] };
            double[] posture = { Relationship[3], Relationship[4], Relationship[5] };
            Quatnum qPosture = AxisAngle2Quatnum(posture);

            double positionLength = LengthOfArray(position);
            double[] positionDirection;
            if (positionLength == 0)
            {
                positionDirection = new double[3] { 0.0, 0.0, 0.0 };
            }
            else
            {
                positionDirection = new double[3] { position[0] / positionLength, position[1] / positionLength, position[2] / positionLength };
            }
            double[] reversePositionDirection = FindDirectionToSecondReferenceFromFirstReference(positionDirection, qPosture);
            double[] reversePosition = { -reversePositionDirection[0] * positionLength, -reversePositionDirection[1] * positionLength, -reversePositionDirection[2] * positionLength };
            double[] reversePosture = Quatnum2AxisAngle(InvQuatnum(qPosture));

            double[] reverseRelationship = { reversePosition[0],reversePosition[1],reversePosition[2],
                                                              reversePosture[0],reversePosture[1],reversePosture[2] };

            return reverseRelationship;
        }

        /// <summary>
        /// 获得第三个坐标系相对于第一个坐标系的位姿，中间隔着第二个坐标系
        /// </summary>
        /// <param name="SecondReferenceToFirstReference">第二个坐标系相对于第一个坐标系的位姿</param>
        /// <param name="TransformFromSecondReferenceToThirdReference">第二个坐标系转换到第三个坐标系的位姿关系</param>
        /// <returns>返回第三个坐标系相对于第一个坐标系的位姿</returns>
        public static double[] FindThirdReferenceToFirstReference(double[] SecondReferenceToFirstReference, double[] TransformFromSecondReferenceToThirdReference)
        {
            // A-->B-->C
            double[] positionAToB = { SecondReferenceToFirstReference[0], SecondReferenceToFirstReference[1], SecondReferenceToFirstReference[2] };
            double[] postureAToB = { SecondReferenceToFirstReference[3], SecondReferenceToFirstReference[4], SecondReferenceToFirstReference[5] };
            Quatnum qPostureAToB = AxisAngle2Quatnum(postureAToB);

            double[] positionBToC = { TransformFromSecondReferenceToThirdReference[0], TransformFromSecondReferenceToThirdReference[1], TransformFromSecondReferenceToThirdReference[2] };
            double positionBToCLength = LengthOfArray(positionBToC);
            double[] positionBToCDirection;
            if (positionBToCLength == 0)
            {
                positionBToCDirection = new double[3] { 0.0, 0.0, 0.0 };
            }
            else
            {
                positionBToCDirection = new double[3] { positionBToC[0] / positionBToCLength, positionBToC[1] / positionBToCLength, positionBToC[2] / positionBToCLength };
            }
            double[] positionBToCDirectionToA = FindDirectionToSecondReferenceFromFirstReference(positionBToCDirection, InvQuatnum(qPostureAToB));
            double[] positionAToC = { positionAToB[0] + positionBToCDirectionToA[0] * positionBToCLength, positionAToB[1] + positionBToCDirectionToA[1] * positionBToCLength, positionAToB[2] + positionBToCDirectionToA[2] * positionBToCLength };

            double[] postureBToC = { TransformFromSecondReferenceToThirdReference[3], TransformFromSecondReferenceToThirdReference[4], TransformFromSecondReferenceToThirdReference[5] };
            double postureBToCAngle = LengthOfArray(postureBToC);
            double[] postureBToCDirection;
            if (postureBToCAngle == 0)
            {
                postureBToCDirection = new double[3] { 0.0, 0.0, 0.0 };
            }
            else
            {
                postureBToCDirection = new double[3] { postureBToC[0] / postureBToCAngle, postureBToC[1] / postureBToCAngle, postureBToC[2] / postureBToCAngle };
            }
            double[] postureBToCDirectionToA = FindDirectionToSecondReferenceFromFirstReference(postureBToCDirection, InvQuatnum(qPostureAToB));
            double[] postureBToCToA = { postureBToCDirectionToA[0] * postureBToCAngle, postureBToCDirectionToA[1] * postureBToCAngle, postureBToCDirectionToA[2] * postureBToCAngle };
            Quatnum qPostureBToCToA = AxisAngle2Quatnum(postureBToCToA);
            Quatnum qPostureAToC = QuatnumRotate(new Quatnum[] { qPostureAToB, qPostureBToCToA });
            double[] postureAToC = Quatnum2AxisAngle(qPostureAToC);

            double[] thirdReferenceToFirstReference = { positionAToC[0], positionAToC[1], positionAToC[2],
                                                                                  postureAToC[0], postureAToC[1], postureAToC[2] };

            return thirdReferenceToFirstReference;
        }

        /// <summary>
        /// 角度转弧度
        /// </summary>
        /// <param name="Deg">角度</param>
        /// <returns>返回弧度</returns>
        public static double Deg2Rad(double Deg)
        {
            return Deg / 180.0 * Math.PI;
        }

        /// <summary>
        /// 弧度转角度
        /// </summary>
        /// <param name="Rad">弧度</param>
        /// <returns>返回角度</returns>
        public static double Rad2Deg(double Rad)
        {
            return Rad / Math.PI * 180.0;
        }

        /// <summary>
        /// 矢量点积
        /// </summary>
        /// <param name="Array1">矢量1</param>
        /// <param name="Array2">矢量2</param>
        /// <returns>返回点积结果</returns>
        public static double VectorDotMultiply(double[] Array1, double[] Array2)
        {
            return Array1[0] * Array2[0] + Array1[1] * Array2[1] + Array1[2] * Array2[2];
        }

        /// <summary>
        /// 矢量叉积
        /// </summary>
        /// <param name="Array1">左矢量</param>
        /// <param name="Array2">右矢量</param>
        /// <returns>返回叉积结果</returns>
        public static double[] VectorCrossMultiply(double[] Array1, double[] Array2)
        {
            return new double[] { Array1[1]*Array2[2]-Array1[2]*Array2[1],
                                              Array1[2]*Array2[0]-Array1[0]*Array2[2],
                                              Array1[0]*Array2[1]-Array1[1]*Array2[0] };
        }

        /// <summary>
        /// 矢量构造
        /// </summary>
        /// <param name="CoefficientOfBaseArray1">构造基矢量1的系数</param>
        /// <param name="CoefficientOfBaseArray2">构造基矢量2的系数</param>
        /// <param name="BaseArray1">构造基矢量1</param>
        /// <param name="BaseArray2">构造基矢量2</param>
        /// <returns></returns>
        public static double[] VectorGeneratedFromBaseArray(double CoefficientOfBaseArray1, double CoefficientOfBaseArray2, double[] BaseArray1, double[] BaseArray2)
        {
            return new double[]{ CoefficientOfBaseArray1 * BaseArray1[0]+CoefficientOfBaseArray2*BaseArray2[0],
                                              CoefficientOfBaseArray1 * BaseArray1[1]+CoefficientOfBaseArray2*BaseArray2[1],
                                              CoefficientOfBaseArray1 * BaseArray1[2]+CoefficientOfBaseArray2*BaseArray2[2] };
        }

        /// <summary>
        /// 自定义列主元法求解3阶线性方程组
        /// </summary>
        /// <param name="Ao">系数矩阵</param>
        /// <param name="bo">列空间投影</param>
        /// <returns>返回解向量</returns>
        public static double[] SolveEqu3Self(double[,] Ao, double[] bo)
        {
            double[,] A = new double[3, 3];
            double[] b = new double[3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    A[i, j] = Ao[i, j];
                }
                b[i] = bo[i];
            }
            int[] seq = new int[3] { 0, 0, 0 };

            for (int k = 1; k < 3; k++)
            {
                if (Math.Abs(A[k, 0]) > Math.Abs(A[seq[0], 0]))
                {
                    seq[0] = k;
                }
            }
            b[seq[0]] = b[seq[0]] / A[seq[0], 0];
            for (int k = 2; k >= 0; k--)
            {
                A[seq[0], k] = A[seq[0], k] / A[seq[0], 0];
            }
            for (int i = 0; i < 3; i++)
            {
                if (i != seq[0])
                {
                    b[i] -= (b[seq[0]] * A[i, 0]);
                    for (int k = 2; k >= 0; k--)
                    {
                        A[i, k] -= (A[seq[0], k] * A[i, 0]);
                    }
                }
            }

            if (seq[0] == 0)
            {
                seq[1] = 1;
            }
            for (int k = 1; k < 3; k++)
            {
                if (k != seq[0] && Math.Abs(A[k, 1]) > Math.Abs(A[seq[1], 1]))
                {
                    seq[1] = k;
                }
            }
            b[seq[1]] = b[seq[1]] / A[seq[1], 1];
            for (int k = 2; k > 0; k--)
            {
                A[seq[1], k] = A[seq[1], k] / A[seq[1], 1];
            }
            for (int i = 0; i < 3; i++)
            {
                if (i != seq[1])
                {
                    b[i] -= (b[seq[1]] * A[i, 1]);
                    for (int k = 2; k > 0; k--)
                    {
                        A[i, k] -= (A[seq[1], k] * A[i, 1]);
                    }
                }
            }

            seq[2] = 3 - (seq[0] + seq[1]);
            b[seq[2]] = b[seq[2]] / A[seq[2], 2];
            A[seq[2], 2] = 1;
            for (int i = 0; i < 3; i++)
            {
                if (i != seq[2])
                {
                    b[i] -= (b[seq[2]] * A[i, 2]);
                    A[i, 2] = 0;
                }
            }

            return new double[] { b[seq[0]], b[seq[1]], b[seq[2]] };
        }

        /// <summary>
        /// 获得采集数据的近似分布，要求数据是离散的，且不小于30个
        /// </summary>
        /// <param name="SampleDatas">采样数据</param>
        /// <returns>统计分布</returns>
        public static Dictionary<double, byte> GetDistributionFromSampleDatas(double[] SampleDatas)
        {
            int size = SampleDatas.Length;
            if (size < 30)
            {
                return null;
            }

            Dictionary<double, byte> statisticalNum = new Dictionary<double, byte>(10);
            foreach (double item in SampleDatas)
            {
                if (statisticalNum.ContainsKey(item))
                {
                    statisticalNum[item]++;
                }
                else
                {
                    statisticalNum.Add(item, 1);
                }
            }
            return statisticalNum;
        }

        /// <summary>
        /// 高斯分布拟合，拟合形式f=Y*exp(-(x-u)^2/S)
        /// </summary>
        /// <param name="SampleDistribution">采样数据的统计分布键值对</param>
        /// <returns>返回拟合参数u，S，Y</returns>
        public static double[] GaussDistributionFit(Dictionary<double, byte> SampleDistribution)
        {
            int catalogNum = SampleDistribution.Count;
            if (catalogNum < 3)
            {
                return null;
            }

            double[] dataPoints = new double[catalogNum];
            byte[] dataNums = new byte[catalogNum];
            SampleDistribution.Keys.CopyTo(dataPoints, 0);
            SampleDistribution.Values.CopyTo(dataNums, 0);

            double maxData = double.MinValue;
            double minData = double.MaxValue;
            int totalNum = 0;
            for (int i = 0; i < catalogNum; i++)
            {
                maxData = (dataPoints[i] > maxData) ? dataPoints[i] : maxData;
                minData = (dataPoints[i] < minData) ? dataPoints[i] : minData;
                totalNum += dataNums[i];
            }
            double dataRange = maxData - minData;

            double[] x = new double[catalogNum];
            double[] pLog = new double[catalogNum];
            for (int i = 0; i < catalogNum; i++)
            {
                x[i] = dataPoints[i] / dataRange;
                pLog[i] = Math.Log(((double)dataNums[i]) / ((double)totalNum));
            }

            double[] b = new double[3];
            double[,] A = new double[3, 3];
            for (int i = 0; i < catalogNum; i++)
            {
                A[1, 0] += x[i];
                A[2, 0] += Math.Pow(x[i], 2);
                A[2, 1] += Math.Pow(x[i], 3);
                A[2, 2] += Math.Pow(x[i], 4);
                b[0] += pLog[i];
                b[1] += x[i] * pLog[i];
                b[2] += (x[i] * x[i]) * pLog[i];
            }
            A[0, 0] = catalogNum;
            A[0, 1] = A[1, 0];
            A[1, 1] = A[2, 0];
            A[0, 2] = A[2, 0];
            A[1, 2] = A[2, 1];

            double[] solutions = SolveEqu3Self(A, b);
            double S = -1 / solutions[2];
            double u = solutions[1] * S / 2.0;
            double Y = Math.Exp(solutions[0] + Math.Pow(u, 2) / S);
            S *= Math.Pow(dataRange, 2);
            u *= dataRange;
            Y *= totalNum;

            if (Double.IsNaN(S) || Double.IsInfinity(S) ||
                Double.IsNaN(u) || Double.IsInfinity(u) ||
                Double.IsNaN(Y) || Double.IsInfinity(Y))
            {
                return null;
            }
            return new double[] { u, S, Y };
        }

        /// <summary>
        /// 获得输入数据的高斯均值，即假设输入参数满足高斯分布时的期望值
        /// 如果输入数据的统计分布不足则线性插值
        /// </summary>
        /// <param name="InputDatas">输入数据</param>
        /// <param name="DecimalNum">输入数据保留位数</param>
        /// <param name="ifAvg">是否直接求均值</param>
        /// <returns>返回高斯均值</returns>
        public static double GaussAverage(double[] InputDatas, int DecimalNum, bool ifAvg = true)
        {
            if (ifAvg)
            {
                double average = 0;
                foreach (double item in InputDatas)
                {
                    average += item;
                }
                return average / (double)InputDatas.Length;
            }

            double[] Datas = new double[InputDatas.Length];
            double decimalNum = Math.Pow(10, (double)DecimalNum);
            for (int i = 0; i < Datas.Length; ++i)
            {
                Datas[i] = Math.Round(InputDatas[i] * decimalNum) / decimalNum;
            }

            Dictionary<double, byte> statisticalNum = GetDistributionFromSampleDatas(Datas);
            if (statisticalNum.Count < 2)
            {
                return Datas[0];
            }
            else if (statisticalNum.Count < 3)
            {
                double average = 0;
                foreach (double item in Datas)
                {
                    average += item;
                }
                return average / (double)Datas.Length;
            }
            else
            {
                double[] gaussValue = GaussDistributionFit(statisticalNum);
                if (Object.Equals(gaussValue, null))
                {
                    double average = 0;
                    foreach (double item in Datas)
                    {
                        average += item;
                    }
                    return average / (double)Datas.Length;
                }
                return gaussValue[0];
            }
        }

        /// <summary>
        /// 简易三次样条插值，使用循环条件的二维插值，要求段数不高于10，首尾节点值相同，等距采样
        /// </summary>
        /// <param name="InterpolatedX">待插值的X坐标，间隔需相等</param>
        /// <param name="InterpolatedY">待插值的Y坐标</param>
        /// <returns>返回插值段参数</returns>
        public static double[,] SimpleCubicSplineCirculatedInterpolation(double[] InterpolatedX, double[] InterpolatedY)
        {
            int nodeDim = InterpolatedX.Length; // 节点个数
            double sampleInterval = InterpolatedX[1] - InterpolatedX[0]; // 采样间隔

            double[] deltaY = new double[nodeDim - 1]; // Y坐标一阶差值
            for (int k = 0; k < nodeDim - 1; k++)
            {
                deltaY[k] = InterpolatedY[k + 1] - InterpolatedY[k];
            }

            double[] outSpace = new double[nodeDim]; // 输出空间
            for (int k = 0; k < nodeDim - 2; k++)
            {
                outSpace[k] = 6.0 * (deltaY[k + 1] - deltaY[k]) / sampleInterval / sampleInterval;
            }
            outSpace[nodeDim - 2] = 6.0 * (deltaY[0] - deltaY[nodeDim - 2]) / sampleInterval / sampleInterval;

            int[] fibonacciParameters = new int[nodeDim - 1]; // 斐波那契型参数
            fibonacciParameters[0] = 1;
            fibonacciParameters[1] = -4;
            for (int k = 2; k < nodeDim - 1; k++)
            {
                fibonacciParameters[k] = -4 * fibonacciParameters[k - 1] - fibonacciParameters[k - 2];
            }

            double[] requiredOutSpace = new double[2] { 0.0, 0.0 }; // 计算中间值的必要输出空间
            for (int k = 0; k < nodeDim - 3; k++)
            {
                requiredOutSpace[0] += (outSpace[k] * fibonacciParameters[k]);
                requiredOutSpace[1] += (outSpace[k + 1] * fibonacciParameters[k]);
            }
            requiredOutSpace[0] += (outSpace[nodeDim - 3] * fibonacciParameters[nodeDim - 3]);
            requiredOutSpace[1] -= outSpace[nodeDim - 2];

            double[] requiredInSpace = new double[4] { -fibonacciParameters[nodeDim - 2], // 计算中间值的必要核空间
                                                                                  fibonacciParameters[nodeDim - 3] + 1.0, 
                                                                                  -fibonacciParameters[nodeDim - 3] - 1.0, 
                                                                                  fibonacciParameters[nodeDim - 4] - 4.0 };

            double[] middleVariables = new double[nodeDim]; // 中间值
            middleVariables[nodeDim - 2] = (requiredOutSpace[0] / requiredInSpace[1] - requiredOutSpace[1] / requiredInSpace[3]) / (requiredInSpace[0] / requiredInSpace[1] - requiredInSpace[2] / requiredInSpace[3]);
            middleVariables[nodeDim - 1] = (requiredOutSpace[0] / requiredInSpace[0] - requiredOutSpace[1] / requiredInSpace[2]) / (requiredInSpace[1] / requiredInSpace[0] - requiredInSpace[3] / requiredInSpace[2]);
            for (int k = nodeDim - 3; k > -1; k--)
            {
                middleVariables[k] = outSpace[k] - 4.0 * middleVariables[k + 1] - middleVariables[k + 2];
            }

            double[,] segmentationParameters = new double[4, nodeDim - 1]; // 分段插值的参数
            for (int k = 0; k < nodeDim - 1; k++)
            {
                segmentationParameters[0, k] = InterpolatedY[k];
                segmentationParameters[1, k] = deltaY[k] / sampleInterval - middleVariables[k] * sampleInterval / 2.0 - (middleVariables[k + 1] - middleVariables[k]) * sampleInterval / 6.0;
                segmentationParameters[2, k] = middleVariables[k] / 2.0;
                segmentationParameters[3, k] = (middleVariables[k + 1] - middleVariables[k]) / 6.0 / sampleInterval;
            }

            return segmentationParameters;
        }

    }
}
