using System;
using System.Collections.Generic;
using System.Reflection;
using LogPrinter;

namespace XMLConnection
{
    /// <summary>
    /// XML文件连接类
    /// </summary>
    public class XMLConnector : XMLBase
    {
        #region 字段
        protected const string fileDirectory = "Conf\\"; // 配置文件所在路径
        protected const string reSaveDirectory = "Saves\\"; // 配置文件转存总路径

        public delegate void SendVoid(); // 无参数发送委托
        public event SendVoid OnSendXmlSaveFailure; // 发送Xml文件保存失败
        public event SendVoid OnSendXmlReplaceFailure; // 发送Xml文件转存失败
        public event SendVoid OnSendXmlRecoverFailure; // 发送Xml文件恢复失败
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="fileName">Xml文件名</param>
        /// <param name="reSaveFileDirectory">转存Xml文件所在的该级目录</param>
        /// <param name="DefaultConfigurations">默认参数配置</param>
        public XMLConnector(string fileName, string reSaveFileDirectory, Dictionary<string, string[]> DefaultConfigurations) :
            base(Convert.ToString(AppDomain.CurrentDomain.BaseDirectory) + fileDirectory + fileName,
                    Convert.ToString(AppDomain.CurrentDomain.BaseDirectory) + fileDirectory + reSaveDirectory + reSaveFileDirectory,
                    DefaultConfigurations) { }

        /// <summary>
        /// 保存参数列表到Xml文件
        /// </summary>
        /// <param name="SaveParameters">保存的参数</param>
        public void SaveXML(Dictionary<string, string[]> SaveParameters)
        {
            bool saveResult = SaveXmlToFile(SaveParameters);

            if (!saveResult)
            {
                OnSendXmlSaveFailure();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Dismatch items appearred in XML file.");
            }
        }

        /// <summary>
        /// 从Xml文件中读取参数列表
        /// </summary>
        /// <returns>返回参数列表</returns>
        public Dictionary<string, string[]> ReadXml()
        {
            return ReadXmlFromFile();
        }

        /// <summary>
        /// 转存Xml文件到指定路径下
        /// </summary>
        /// <param name="FileName">转存后的文件名</param>
        public void ReplaceXml(string FileName)
        {
            bool reSaveResult = ReSaveXmlToFile(FileName);

            if (!reSaveResult)
            {
                OnSendXmlReplaceFailure();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Can not find orignal XML file.");
            }
        }

        /// <summary>
        /// 恢复转存路径下的Xml文件到主路径下
        /// </summary>
        /// <param name="FileName">需要恢复的文件名</param>
        public void RecoverXml(string FileName)
        {
            bool recoverResult = ReReadXmlToFile(FileName);

            if (!recoverResult)
            {
                OnSendXmlRecoverFailure();
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Can not find replaced XML file.");
            }
        }
        #endregion
    }
}
