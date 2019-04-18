using System;
using System.Collections.Generic;
using System.IO;
using System.Xml;

namespace XMLConnection
{
    /// <summary>
    /// XML读写基类
    /// </summary>
    public class XMLBase
    {
        #region 字段
        protected string filePath = ""; // xml文件路径
        protected string reSavePathOnly = ""; // xml文件转存路径，不包括文件名
        protected Dictionary<string, string[]> initialParameters; // 参数默认值
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="FilePath">Xml文件路径</param>
        /// <param name="ReSavePathOnly">Xml文件转存路径，不包括文件名</param>
        /// <param name="InitialParameters">参数默认值</param>
        public XMLBase(string FilePath, string ReSavePathOnly, Dictionary<string, string[]> InitialParameters)
        {
            // Xml文件路径赋值
            filePath = FilePath;

            // Xml文件转存路径赋值
            reSavePathOnly = ReSavePathOnly;

            // 参数默认值赋值
            initialParameters = new Dictionary<string, string[]>(InitialParameters.Count);
            foreach (var item in InitialParameters)
            {
                initialParameters.Add(item.Key, (string[])item.Value.Clone());
            }
        }

        /// <summary>
        /// 保存参数列表到Xml文件
        /// </summary>
        /// <param name="SaveParameters">保存的参数，无输入则表示按默认值创建</param>
        /// <returns>返回保存结果</returns>
        protected bool SaveXmlToFile(Dictionary<string, string[]> SaveParameters = null)
        {
            XmlDocument xmlFile = new XmlDocument();
            XmlDeclaration xmlHead = xmlFile.CreateXmlDeclaration("1.0", "UTF-8", null);
            xmlFile.AppendChild(xmlHead);
            XmlElement xmlRoot = xmlFile.CreateElement("Configuration");
            xmlFile.AppendChild(xmlRoot);

            if (Object.Equals(SaveParameters, null))
            {
                SaveParameters = initialParameters;
            }

            foreach (var item in SaveParameters)
            {
                if (item.Value.Length == 1)
                {
                    AddXmlValue(xmlFile, xmlRoot, item.Key, item.Value[0]);
                }
                else if (item.Value.Length > 1)
                {
                    AddXmlArray(xmlFile, xmlRoot, item.Key, item.Value);
                }
                else
                {
                    return false;
                }
            }
            xmlFile.Save(filePath);

            return true;
        }

        /// <summary>
        /// 从Xml文件中读取参数列表
        /// </summary>
        /// <returns>返回参数列表</returns>
        protected Dictionary<string, string[]> ReadXmlFromFile()
        {
            Dictionary<string, string[]> returnBackParameters = new Dictionary<string, string[]>(initialParameters.Count);

            if (!File.Exists(filePath))
            {
                SaveXmlToFile();
                foreach (var item in initialParameters)
                {
                    returnBackParameters.Add(item.Key, (string[])item.Value.Clone());
                }
                return returnBackParameters;
            }

            XmlDocument xmlFile = new XmlDocument();
            xmlFile.Load(filePath);
            XmlElement xmlRoot = xmlFile.DocumentElement;
            XmlNodeList xmlNodes = xmlRoot.ChildNodes;

            foreach (XmlNode item in xmlNodes)
            {
                if (item.ChildNodes.Count > 1)
                {
                    List<string> parameterStrings = new List<string>(item.ChildNodes.Count);

                    XmlNodeList itemNodes = item.ChildNodes;
                    foreach (XmlNode node in itemNodes)
                    {
                        XmlElement nodeElement = (XmlElement)node;
                        parameterStrings.Add(nodeElement.InnerText);
                    }

                    returnBackParameters.Add(item.Name, parameterStrings.ToArray());
                }
                else
                {
                    XmlElement itemElement = (XmlElement)item;
                    returnBackParameters.Add(item.Name, new string[]{item.InnerText});
                }
            }

            return returnBackParameters;
        }

        /// <summary>
        /// 增加单项参数
        /// </summary>
        /// <param name="XmlFile">Xml文件</param>
        /// <param name="XmlRoot">Xml根节点</param>
        /// <param name="Name">参数名称</param>
        /// <param name="Value">参数值</param>
        protected void AddXmlValue(XmlDocument XmlFile, XmlElement XmlRoot, string Name, string Value)
        {
            XmlElement tempElement = XmlFile.CreateElement(Name);
            tempElement.InnerText = Value;
            XmlRoot.AppendChild(tempElement);
        }

        /// <summary>
        /// 增加多项参数
        /// </summary>
        /// <param name="XmlFile">Xml文件</param>
        /// <param name="XmlRoot">Xml根节点</param>
        /// <param name="Name">参数名称</param>
        /// <param name="Values">参数值</param>
        protected void AddXmlArray(XmlDocument XmlFile, XmlElement XmlRoot, string Name, string[] Values)
        {
            XmlNode tempNode = XmlFile.CreateElement(Name);
            int length = Values.Length;
            for (int mask = 0; mask < length; mask++)
            {
                XmlElement tempElement = XmlFile.CreateElement("item" + mask.ToString());
                tempElement.InnerText = Values[mask];
                tempNode.AppendChild(tempElement);
            }
            XmlRoot.AppendChild(tempNode);
        }

        /// <summary>
        /// 转存Xml文件到指定路径下
        /// </summary>
        /// <param name="FileName">转存后的文件名</param>
        /// <returns>返回转存结果</returns>
        protected bool ReSaveXmlToFile(string FileName)
        {
            if (!File.Exists(filePath))
            {
                return false;
            }

            File.Copy(filePath, reSavePathOnly + FileName, true);

            return true;
        }

        /// <summary>
        /// 恢复转存路径下的Xml文件到主路径下
        /// </summary>
        /// <param name="FileName">需要恢复的文件名</param>
        /// <returns>返回恢复结果</returns>
        protected bool ReReadXmlToFile(string FileName)
        {
            if (!File.Exists(reSavePathOnly + FileName))
            {
                return false;
            }

            File.Copy(reSavePathOnly + FileName, filePath, true);

            return true;
        }
        #endregion
    }
}
