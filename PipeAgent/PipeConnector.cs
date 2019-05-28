using System;
using System.Collections.Generic;
//using System.Linq;
//using System.Text;
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

        protected bool ifSendEnabled = false;
        protected bool ifRecieveEnabled = false;

        protected Task sendTask;
        protected CancellationTokenSource sendCancel;
        protected Task recieveTask;
        protected CancellationTokenSource recieveCancel;

        public delegate void SendByteArray(); // 字节流传送委托

        /// <summary>
        /// 传送接收到的字节流
        /// </summary>
        public event SendByteArray OnSendByteRecieved;

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
                if (ifSendEnabled) sendQueue.Enqueue(bytes);
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

                SendByteDatas(dataBytes);
            }

            ifSendEnabled = false;
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

                SendByteDatas(dataBytes);
            }

            ifRecieveEnabled = false;
            Logger.HistoryPrinting(Logger.Level.INFO, MethodBase.GetCurrentMethod().DeclaringType.FullName, "End to recieve bytes at client side of pipe.");
        }
        #endregion
    }
}
