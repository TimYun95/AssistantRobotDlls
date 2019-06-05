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
        protected NamedPipeClientStream innerPipe;

        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="pipeName">管道名称</param>
        public PipeBase(string pipeName)
        {
            innerPipe = new NamedPipeClientStream(".", pipeName, PipeDirection.InOut, PipeOptions.None);
        }

        /// <summary>
        /// 连接管道
        /// </summary>
        /// <returns>返回连接结果</returns>
        public virtual bool PipeConnect()
        {
            try
            {
                innerPipe.Connect(200);
            }
            catch (TimeoutException ex)
            {
                Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Pipe can not be connected.", ex);
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
            innerPipe.Write(datas, 0, datas.Length);
            innerPipe.Flush();
        }

        /// <summary>
        /// 读字节流
        /// </summary>
        /// <returns>返回读取的字节流</returns>
        protected byte[] ReadByteDatas()
        {
            byte[] buf = new byte[128];
            int length = innerPipe.Read(buf, 0, 128);

            return buf.Take(length).ToArray();
        }

        /// <summary>
        /// 关闭管道连接
        /// </summary>
        protected void PipeOver()
        {
            innerPipe.Close();
        }
        #endregion
    }
}
