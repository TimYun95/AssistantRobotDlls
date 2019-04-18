using System;
using System.Collections.Generic;
using System.Reflection;
using System.Data.SqlClient;
using LogPrinter;

namespace SQLServerConnection
{
    /// <summary>
    /// 与SQL Server数据交换基类
    /// </summary>
    public class SQLServerExchangeBase
    {
        #region 字段
        protected string dataBaseDocument = Environment.CurrentDirectory + "\\DataBase\\ToolDataBase.mdf"; // 数据库文件位置

        public delegate void SendVoid(); // 无参数发送委托

        /// <summary>
        /// 发送数据库无法连接到消息
        /// </summary>
        public event SendVoid OnSendDataBaseNotAttached;
        #endregion

        #region 属性
        /// <summary>
        /// 数据库文件位置
        /// </summary>
        public string DataBaseDocument
        {
            get { return dataBaseDocument; }
            set { dataBaseDocument = value; }
        }
        #endregion

        #region 方法
        /// <summary>
        /// 对数据库执行无应答命令
        /// </summary>
        /// <param name="NoReplyCommandStr">无应答命令</param>
        /// <returns>执行是否成功</returns>
        protected bool SendNoReplyCommandToDataBase(string NoReplyCommandStr)
        {
            using (SqlConnection sqlConnection = new SqlConnection(
                      "Data Source=(LocalDB)\\v11.0;" +
                      "AttachDbFilename=" + dataBaseDocument +
                      ";Integrated Security=True;"))
            {
                try
                {
                    sqlConnection.Open();
                }
                catch (Exception ex)
                {
                    OnSendDataBaseNotAttached();
                    Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "DataBase connection can not be attached when try to connect the database.", ex);
                    
                    return false;
                }

                using (SqlCommand sqlCommand = new SqlCommand(NoReplyCommandStr, sqlConnection))
                {
                    try
                    {
                        if (sqlCommand.ExecuteNonQuery() != -1)
                        {
                            return true;
                        }
                        else
                        {
                            Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "DataBase has some problems with datas.");
                            return false;
                        }
                    }
                    catch (Exception ex)
                    {
                        OnSendDataBaseNotAttached();
                        Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "DataBase connection can not be attached when try to send only to database.", ex);

                        return false;
                    }
                }
            }
        }

        /// <summary>
        /// 对数据库执行有应答命令
        /// </summary>
        /// <param name="WithReplyCommandStr">有应答命令</param>
        /// <returns>应答结果</returns>
        protected object[] SendWithReplyCommandToDataBase(string WithReplyCommandStr)
        {
            using (SqlConnection sqlConnection = new SqlConnection(
                      "Data Source=(LocalDB)\\v11.0;" +
                      "AttachDbFilename=" + dataBaseDocument +
                      ";Integrated Security=True;"))
            {
                try
                {
                    sqlConnection.Open();
                }
                catch (Exception ex)
                {
                    OnSendDataBaseNotAttached();
                    Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "DataBase connection can not be attached when try to connect the database.", ex);
                    return new object[] { };
                }

                using (SqlCommand sqlCommand = new SqlCommand(WithReplyCommandStr, sqlConnection))
                {
                    try
                    {
                        using (SqlDataReader sqlReader = sqlCommand.ExecuteReader())
                        {
                            List<object> resultList = new List<object>(200);

                            while (sqlReader.Read())
                            {
                                if (sqlReader.FieldCount < 1 || !sqlReader.HasRows)
                                {
                                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "No enough datas in the specific table.");
                                    return new object[] { };
                                }

                                for (int i = 0; i < sqlReader.FieldCount; i++)
                                {
                                    resultList.Add(sqlReader.GetValue(i));
                                }
                            }

                            return resultList.ToArray();
                        }
                    }
                    catch (Exception ex)
                    {
                        OnSendDataBaseNotAttached();
                        Logger.HistoryPrinting(Logger.Level.ERROR, MethodBase.GetCurrentMethod().DeclaringType.FullName, "DataBase connection can not be attached when try to send and listen to database.", ex);
                        return new object[] { };
                    }
                }
            }
        }
        #endregion
    }





}
