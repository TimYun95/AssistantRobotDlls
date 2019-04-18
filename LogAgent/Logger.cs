using System;
using System.Collections.Generic;
using System.IO;
using log4net;

namespace LogPrinter
{
    /// <summary>
    /// 日志记录类
    /// </summary>
    public class Logger
    {
        #region 枚举
        /// <summary>
        /// 历史日志等级
        /// </summary>
        public enum Level : short
        {
            FATAL = 0,
            ERROR,
            WARN,
            INFO,
            DEBUG
        }
        #endregion

        #region 字段
        protected static log4net.ILog historyLogger = log4net.LogManager.GetLogger("historylogger");
        protected const string dataPath = "Log\\Data\\";
        #endregion

        #region 方法
        /// <summary>
        /// 私有构造函数，静止类外实例
        /// </summary>
        private Logger() { }

        /// <summary>
        /// 历史日志输出
        /// </summary>
        /// <param name="HistoryLevel">历史日志等级</param>
        /// <param name="ClassName">所在类名称</param>
        /// <param name="Content">历史日志内容</param>
        /// <param name="Abnormality">截获的异常，默认为null</param>
        public static void HistoryPrinting(Level HistoryLevel, string ClassName, string Content, Exception Abnormality = null)
        {
            string exceptionStr = "";
            if (!Object.Equals(Abnormality, null))
            {
                exceptionStr = "\r\nException: " + Abnormality.ToString();
            }

            switch (HistoryLevel)
            {
                case Level.FATAL:
                    historyLogger.Fatal(ClassName + ":" + Content + exceptionStr);
                    break;
                case Level.ERROR:
                    historyLogger.Error(ClassName + ":" + Content + exceptionStr);
                    break;
                case Level.WARN:
                    historyLogger.Warn(ClassName + ":" + Content + exceptionStr);
                    break;
                case Level.INFO:
                    historyLogger.Info(ClassName + ":" + Content + exceptionStr);
                    break;
                case Level.DEBUG:
                    historyLogger.Debug(ClassName + ":" + Content + exceptionStr);
                    break;
            }
        }

        /// <summary>
        /// 内存中的数据以日志内容输出
        /// </summary>
        /// <param name="PrintDatas">输出内容</param>
        /// <param name="IsReverse">安装模式</param>
        /// <param name="TcpCordinate">安装TCP坐标</param>
        /// <param name="Mass">末端重量</param>
        public static void DataPrinting(List<double[]> PrintDatas, bool? IsReverse = null, double[] TcpCordinate = null, double? Mass = null)
        {
            string dataFullPath = Convert.ToString(System.AppDomain.CurrentDomain.BaseDirectory) + dataPath + DateTime.Now.ToString("yyyy-MM-dd") + "\\";
            if (!Directory.Exists(dataFullPath))
            {
                Directory.CreateDirectory(dataFullPath);
            }

            string dataFullPathTxt = dataFullPath + DateTime.Now.Hour.ToString() + "_" + DateTime.Now.Minute.ToString() + "_" + DateTime.Now.Second.ToString();
            using (FileStream fs = new FileStream(dataFullPathTxt, FileMode.Create))
            {
                int printLength = PrintDatas.Count - 1;
                List<string> printStr = new List<string>(printLength + 5);

                // Head Part
                if (IsReverse.HasValue)
                {
                    printStr.Add("安装模式: " + (IsReverse.Value ? "倒装" : "正装"));
                }
                else
                {
                    printStr.Add("安装模式: Unknown");
                }
                if (!Object.Equals(TcpCordinate, null))
                {
                    printStr.Add("Tcp坐标: [ " + (TcpCordinate[0] * 1000.0).ToString("0.00") + " " +
                                                                (TcpCordinate[1] * 1000.0).ToString("0.00") + " " +
                                                                (TcpCordinate[2] * 1000.0).ToString("0.00") + " " +
                                                                TcpCordinate[3].ToString("0.0000") + " " +
                                                                TcpCordinate[4].ToString("0.0000") + " " +
                                                                TcpCordinate[5].ToString("0.0000") + " ] (mm, rad)");
                }
                else
                {
                    printStr.Add("Tcp坐标: Unknown");
                }
                if (Mass.HasValue)
                {
                    printStr.Add("Tcp重量: " + Mass.Value.ToString("0.00") + " (kg)");
                }
                else
                {
                    printStr.Add("Tcp重量: Unknown");
                }

                printStr.Add("模块代号: " + PrintDatas[0][0].ToString("0"));

                string customedParameterString = "部分配置参数:";
                for (int k = 1; k < PrintDatas[0].Length; k++)
                {
                    customedParameterString += (" " + PrintDatas[0][k].ToString("0.0000"));
                }
                printStr.Add(customedParameterString);

                printStr.Add("工具X坐标(mm) 工具Y坐标(mm) 工具Z坐标(mm) 工具RX坐标(rad) 工具RY坐标(rad) 工具RZ坐标(rad) " +
                                    "X方向参考力(N) Y方向参考力(N) Z方向参考力(N) RX方向参考力矩(Nm) RY方向参考力矩(Nm) RZ方向参考力矩(Nm) " +
                                    "其余自定义参数...");

                // Content Part
                int printUnitLength = PrintDatas[1].Length;
                for (int i = 1; i < PrintDatas.Count; i++)
                {
                    string requireStr = (PrintDatas[i][0] * 1000.0).ToString("0.00") + " " +
                                                  (PrintDatas[i][1] * 1000.0).ToString("0.00") + " " +
                                                  (PrintDatas[i][2] * 1000.0).ToString("0.00") + " " +
                                                  PrintDatas[i][3].ToString("0.0000") + " " +
                                                  PrintDatas[i][4].ToString("0.0000") + " " +
                                                  PrintDatas[i][5].ToString("0.0000") + " " +
                                                  PrintDatas[i][6].ToString("0.00") + " " +
                                                  PrintDatas[i][7].ToString("0.00") + " " +
                                                  PrintDatas[i][8].ToString("0.00") + " " +
                                                  PrintDatas[i][9].ToString("0.000") + " " +
                                                  PrintDatas[i][10].ToString("0.000") + " " +
                                                  PrintDatas[i][11].ToString("0.000");
                    for (int n = 12; n < printUnitLength; n++)
                    {
                        requireStr += (" " + PrintDatas[i][n].ToString("0.0000"));
                    }
                    printStr.Add(requireStr);
                }

                // Output
                using (StreamWriter sw = new StreamWriter(fs))
                {
                    sw.Write(String.Join("\r\n", printStr.ToArray()));
                    sw.Flush();
                }
            }



















            //string message = (ToolPositions[0] * 1000).ToString("0.00") + " " +
            //                            (ToolPositions[1] * 1000).ToString("0.00") + " " +
            //                            (ToolPositions[2] * 1000).ToString("0.00") + " " +
            //                            ToolPositions[3].ToString("0.0000") + " " +
            //                            ToolPositions[4].ToString("0.0000") + " " +
            //                            ToolPositions[5].ToString("0.0000") + " " +
            //                            SixAxisForces[0].ToString("0.00") + " " +
            //                            SixAxisForces[1].ToString("0.00") + " " +
            //                            SixAxisForces[2].ToString("0.00") + " " +
            //                            SixAxisForces[3].ToString("0.000") + " " +
            //                            SixAxisForces[4].ToString("0.000") + " " +
            //                            SixAxisForces[5].ToString("0.000");
            //datalogger.Info(message);
        }
        #endregion





    }
}
