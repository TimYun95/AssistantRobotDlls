using System;
using System.IO;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Reflection;

using LogPrinter;

namespace PipeCommunication
{
    /// <summary>
    /// 管道连接类
    /// </summary>
    public class PipeConnector : PipeBase
    {
        #region 字段
        protected const int queueMaxLength = 10;

        protected Queue<byte[]> sendQueue;
        private static readonly object sendLocker = new object();
        protected const int sleepMsForQueueSend = 10;

        protected bool ifSendEnabled = false; // 是否允许发送
        protected bool ifRecieveEnabled = false; // 是否允许接收

        protected Task sendTask;
        protected CancellationTokenSource sendCancel;
        protected Task recieveTask;
        protected CancellationTokenSource recieveCancel;

        public delegate void SendByteArray(byte[] sendBytes); // 字节流传送委托

        /// <summary>
        /// 传送接收到的字节流
        /// </summary>
        public event SendByteArray OnSendByteRecieved;

        public delegate void SendVoid(); // 空传送委托

        /// <summary>
        /// 传送管道终止状态
        /// </summary>
        public event SendVoid OnSendPipeCrashed;
        #endregion

        #region 方法
        /// <summary>
        /// 构造函数
        /// </summary>
        /// <param name="pipeName">管道名称</param>
        public PipeConnector(string pipeName) :
            base(pipeName)
        {
            sendQueue.Clear();
        }

        /// <summary>
        /// 连接管道
        /// </summary>
        /// <returns>返回连接结果</returns>
        public override bool PipeConnect()
        {
            if (!base.PipeConnect()) return false;
            else
            {
                sendQueue.Clear();

                sendCancel = new CancellationTokenSource();
                recieveCancel = new CancellationTokenSource();

                sendTask = new Task(() => sendTaskWork(sendCancel.Token));
                recieveTask = new Task(() => recieveTaskWork(recieveCancel.Token));

                sendTask.Start();
                recieveTask.Start();

                return true;
            }
        }

        /// <summary>
        /// 发送字节流
        /// </summary>
        /// <param name="bytes">字节流</param>
        public void SendBytes(byte[] bytes)
        {
            lock (sendLocker)
            {
                if (ifSendEnabled)
                {
                    if (sendQueue.Count < queueMaxLength) sendQueue.Enqueue(bytes);
                    else Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Send queue is full.");
                }
            }
        }

        /// <summary>
        /// 字节流发送任务
        /// </summary>
        /// <param name="cancelFlag">取消标志</param>
        protected void sendTaskWork(CancellationToken cancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Ready to send bytes at client side of pipe.");
            ifSendEnabled = true;

            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                Thread.Sleep(sleepMsForQueueSend);

                byte[] dataBytes = null;
                lock (sendLocker)
                {
                    if (sendQueue.Count > 0)
                    {
                        dataBytes = sendQueue.Dequeue();
                    }
                }

                if (dataBytes.Equals(null)) continue;

                try
                {
                    SendByteDatas(dataBytes);
                }
                catch (IOException ex)
                {
                    sendCancel.Cancel();
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Pipe crashed when sending bytes.", ex);
                }
            }

            ifSendEnabled = false;
            recieveCancel.Cancel();

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End to send bytes at client side of pipe.");
        }

        /// <summary>
        /// 字节流接收任务
        /// </summary>
        /// <param name="cancelFlag">取消标志</param>
        protected void recieveTaskWork(CancellationToken cancelFlag)
        {
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Ready to recieve bytes at client side of pipe.");
            ifRecieveEnabled = true;

            while (true)
            {
                if (cancelFlag.IsCancellationRequested) break;

                byte[] dataBytes = ReadByteDatas();

                if (dataBytes.Length < 1)
                {
                    Logger.HistoryPrinting(Logger.Level.WARN, MethodBase.GetCurrentMethod().DeclaringType.FullName, "Pipe crashed when recieving bytes.");
                    Thread.Sleep(sleepMsForQueueSend);
                }
                else OnSendByteRecieved(dataBytes);
            }

            ifRecieveEnabled = false;
            OnSendPipeCrashed();

            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End to recieve bytes at client side of pipe.");
        }

        #endregion
    }
}
