using System;
using System.IO.Pipes;
using System.Reflection;
using System.Linq;

using LogPrinter;

namespace PipeCommunication
{
    /// <summary>
    /// 管道基类
    /// </summary>
    public class PipeBase
    {
        #region 字段
        protected NamedPipeClientStream innerPipeWriteToServer;
        protected NamedPipeClientStream innerPipeReadFromServer;

        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        public PipeBase()
        {
            innerPipeWriteToServer = new NamedPipeClientStream(".", "pipeWriteToServer", PipeDirection.Out);
            innerPipeReadFromServer = new NamedPipeClientStream(".", "pipeReadFromServer", PipeDirection.In);
        }

        /// <summary>
        /// 连接管道
        /// </summary>
        /// <returns>返回连接结果</returns>
        public virtual bool PipeConnect()
        {
            try
            {
                innerPipeWriteToServer.Connect(250);
                innerPipeReadFromServer.Connect(250);
            }
            catch (TimeoutException ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Pipe can not be connected.", ex);
                return false;
            }
            catch (Exception ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Pipe can not be connected, unknown error.", ex);
                return false;
            }
            return true;
        }

        /// <summary>
        /// 写字节流
        /// </summary>
        /// <param name="datas">字节流</param>
        protected void SendByteDatas(byte[] datas)
        {
            innerPipeWriteToServer.Write(datas, 0, datas.Length);
            innerPipeWriteToServer.Flush();
        }

        /// <summary>
        /// 读字节流
        /// </summary>
        /// <returns>返回读取的字节流</returns>
        protected byte[] ReadByteDatas()
        {
            byte[] buf = new byte[128];
            int length = innerPipeReadFromServer.Read(buf, 0, 128);

            return buf.Take(length).ToArray();
        }

        /// <summary>
        /// 关闭管道连接
        /// </summary>
        protected void PipeOver()
        {
            innerPipeReadFromServer.Close();
            innerPipeWriteToServer.Close();
        }
        #endregion
    }
}
