using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using LogPrinter;

namespace ResourceCheck
{
    /// <summary>
    /// 资源检测类，检查相应资源是否齐备
    /// </summary>
    public class ResourceChecker
    {
        #region 字段
        private const string controllerCodePath = "ControllerCode\\";
        private const string controllerCodeFilePath = "ControllerCode\\ControllerCode.txt";

        private const string dataBasePath = "DataBase\\";
        private const string dataBaseFilePath = "DataBase\\ToolDataBase.mdf";

        private const string logPath = "Log\\";
        private const string logHistoryPath = "Log\\History\\";
        private const string logDataPath = "Log\\Data\\";
        private const string logConfFilePath = "Log\\Log.config";

        private const string confPath = "Conf\\";
        private const string confSavePath = "Conf\\Saves\\";
        private const string defaultGalactophoreConfPath = "Conf\\Saves\\GalactophoreDetection\\";

        private const string keyDataPath = "SavedKeyDatas\\";
        #endregion

        #region 方法
        /// <summary>
        /// 私有构造函数，静止类外实例
        /// </summary>
        private ResourceChecker() { }

        /// <summary>
        /// 检测相关资源是否存在
        /// </summary>
        /// <param name="modelConfTransferPath">所需的模块配置文件转存路径</param>
        /// <param name="ifIncludeKeyDataPath">是否包含关键数据保存路径</param>
        /// <returns>返回检查结果</returns>
        public static bool ResourceChecking(string[] modelConfTransferPath = null, bool ifIncludeKeyDataPath = false)
        {
            bool checkingResult = true;

            // 判断为空则使用默认值
            if (object.Equals(modelConfTransferPath, null))
            {
                modelConfTransferPath = new string[] { defaultGalactophoreConfPath };
            }

            // 不存在日志文件夹则创建
            if (!Directory.Exists(logPath))
            {
                try
                {
                    Directory.CreateDirectory(logPath);
                    Directory.CreateDirectory(logHistoryPath);
                    Directory.CreateDirectory(logDataPath);

                    CreateLogConf();
                }
                catch (Exception ex)
                {
                    checkingResult = false;
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create log, its sub directories and configuration failed.", ex);
                }
            }
            else
            {
                // 不存在历史日志文件夹则创建
                if (!Directory.Exists(logHistoryPath))
                {
                    try
                    {
                        Directory.CreateDirectory(logHistoryPath);
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create history log directory failed.", ex);
                    }
                }
                // 不存在数据日志文件夹则创建
                if (!Directory.Exists(logDataPath))
                {
                    try
                    {
                        Directory.CreateDirectory(logDataPath);
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create data log directory failed.", ex);
                    }
                }
                // 不存在日志配置文件则创建
                if (!File.Exists(logConfFilePath))
                {
                    try
                    {
                        CreateLogConf();
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create log configuration failed.", ex);
                    }
                }
            }

            // 不存在控制程序文件夹则创建
            if (!Directory.Exists(controllerCodePath))
            {
                try
                {
                    Directory.CreateDirectory(controllerCodePath);
                }
                catch (Exception ex)
                {
                    checkingResult = false;
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create controlling code directory failed.", ex);
                }
            }

            // 不存在数据库文件夹则创建
            if (!Directory.Exists(dataBasePath))
            {
                try
                {
                    Directory.CreateDirectory(dataBasePath);
                    using (FileStream fs = new FileStream(dataBaseFilePath, FileMode.Create))
                    {
                        byte[] toolDB = ResourceAgent.Properties.Resources.ToolDataBase;
                        fs.Write(toolDB, 0, toolDB.Length);
                    }
                }
                catch (Exception ex)
                {
                    checkingResult = false;
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create database directory and file failed.", ex);
                }
            }
            else
            {
                // 不存在数据库文件则创建
                if (!File.Exists(dataBaseFilePath))
                {
                    try
                    {
                        using (FileStream fs = new FileStream(dataBaseFilePath, FileMode.Create))
                        {
                            byte[] toolDB = ResourceAgent.Properties.Resources.ToolDataBase;
                            fs.Write(toolDB, 0, toolDB.Length);
                        }
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create database file failed.", ex);
                    }
                }
            }

            // 不存在控制配置文件夹则创建
            if (!Directory.Exists(confPath))
            {
                try
                {
                    Directory.CreateDirectory(confPath);
                    Directory.CreateDirectory(confSavePath);
                    foreach (string path in modelConfTransferPath)
                    {
                        Directory.CreateDirectory(path);
                    }
                }
                catch (Exception ex)
                {
                    checkingResult = false;
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create conf, conf transfer and each module conf directories failed.", ex);
                }
            }
            else
            {
                // 不存在控制配置转存文件夹则创建
                if (!Directory.Exists(confSavePath))
                {
                    try
                    {
                        Directory.CreateDirectory(confSavePath);
                        foreach (string path in modelConfTransferPath)
                        {
                            Directory.CreateDirectory(path);
                        }
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create conf transfer and each module conf directories failed.", ex);
                    }
                }
                else
                {
                    // 不存在模块控制配置转存文件夹则创建
                    foreach (string path in modelConfTransferPath)
                    {
                        try
                        {
                            if (!Directory.Exists(path))
                            {
                                Directory.CreateDirectory(path);
                            }
                        }
                        catch (Exception ex)
                        {
                            checkingResult = false;
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create module conf directory \"" + path + "\" failed.", ex);
                        }
                    }
                }
            }

            if (ifIncludeKeyDataPath)
            {
                // 不存在关键数据保存文件夹则创建
                if (!Directory.Exists(keyDataPath))
                {
                    try
                    {
                        Directory.CreateDirectory(keyDataPath);
                    }
                    catch (Exception ex)
                    {
                        checkingResult = false;
                        Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Create key data directory failed.", ex);
                    }
                }
            }

            return checkingResult;
        }

        /// <summary>
        /// 创建Log.config文件
        /// </summary>
        protected static void CreateLogConf()
        {
            using (FileStream fileStream = new FileStream(logConfFilePath, FileMode.Create))
            {
                using (StreamWriter streamWriter = new StreamWriter(fileStream))
                {
                    List<string> WriteStrs = new List<string>(40);
                    WriteStrs.Add("<?xml version=\"1.0\" encoding=\"utf-8\" ?>\r\n");
                    WriteStrs.Add("<configuration>\r\n");
                    WriteStrs.Add("  <configSections>\r\n");
                    WriteStrs.Add("    <section name=\"log4net\" type=\"log4net.Config.Log4NetConfigurationSectionHandler, log4net\" />\r\n");
                    WriteStrs.Add("  </configSections>\r\n");
                    WriteStrs.Add("  <log4net>\r\n");
                    WriteStrs.Add("    <logger name=\"historylogger\">\r\n");
                    WriteStrs.Add("      <!--历史日志记录-->\r\n");
                    WriteStrs.Add("      <level value=\"ALL\" />\r\n");
                    WriteStrs.Add("      <appender-ref ref=\"RollingLogFileHistoryAppender\" />\r\n");
                    WriteStrs.Add("    </logger>\r\n");
                    WriteStrs.Add("    <appender name=\"RollingLogFileHistoryAppender\" type=\"log4net.Appender.RollingFileAppender\">\r\n");
                    WriteStrs.Add("      <param name=\"File\" value=\"Log/History/\" />\r\n");
                    WriteStrs.Add("      <!--日志存放路径-->\r\n");
                    WriteStrs.Add("      <param name=\"AppendToFile\" value=\"true\" />\r\n");
                    WriteStrs.Add("      <!--添加到文件末-->\r\n");
                    WriteStrs.Add("      <lockingModel type=\"log4net.Appender.FileAppender+MinimalLock\" />\r\n");
                    WriteStrs.Add("      <!--允许多线程写入，但线程不安全-->\r\n");
                    WriteStrs.Add("      <Encoding value=\"UTF-8\" />\r\n");
                    WriteStrs.Add("      <param name=\"MaxSizeRollBackups\" value=\"-1\" />\r\n");
                    WriteStrs.Add("      <!--最多产生的文件数，-1为无限个-->\r\n");
                    WriteStrs.Add("      <param name=\"StaticLogFileName\" value=\"false\" />\r\n");
                    WriteStrs.Add("      <!--是否只写到一个文件中-->\r\n");
                    WriteStrs.Add("      <param name=\"RollingStyle\" value=\"Composite\" />\r\n");
                    WriteStrs.Add("      <!--按照何种方式产生多个日志文件(日期[Date],文件大小[Size],混合[Composite])-->\r\n");
                    WriteStrs.Add("      <param name=\"DatePattern\" value=\"yyyy-MM-dd/yyyy-MM-dd-&quot;history.log&quot;\"  />\r\n");
                    WriteStrs.Add("      <!--此处按日期产生文件夹，文件名固定。注意&quot; 的位置-->\r\n");
                    WriteStrs.Add("      <param name=\"maximumFileSize\" value=\"500KB\" />\r\n");
                    WriteStrs.Add("      <!--单个文件最大尺寸-->\r\n");
                    WriteStrs.Add("      <param name=\"CountDirection\" value=\"1\"/>\r\n");
                    WriteStrs.Add("      <layout type=\"log4net.Layout.PatternLayout\">\r\n");
                    WriteStrs.Add("        <param name=\"ConversionPattern\" value=\"[%date{yyyy-MM-dd HH:mm:ss,fff}] %level %thread:%message%newline\" />\r\n");
                    WriteStrs.Add("      </layout>\r\n");
                    WriteStrs.Add("    </appender>\r\n");
                    WriteStrs.Add("  </log4net>\r\n");
                    WriteStrs.Add("</configuration>");

                    streamWriter.Write(string.Join("", WriteStrs.ToArray()));
                    streamWriter.Flush();
                }
            }
        }
        #endregion
    }
}
