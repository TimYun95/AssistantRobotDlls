﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SQLServerConnection
{
    /// <summary>
    /// SQL Server连接器
    /// </summary>
    public class SQLServerConnector : SQLServerExchanger
    {
        #region 方法
        /// <summary>
        /// 查找当前工具数目
        /// </summary>
        /// <returns>返回当前工具数目</returns>
        public virtual int GetExistToolNumber()
        {
            return AskCurrentItemLength("dbo.ToolBase");
        }

        /// <summary>
        /// 增加新工具
        /// </summary>
        /// <param name="AddInformation">新工具信息</param>
        /// <returns>新工具分配的工具号</returns>
        public virtual int AddNewTool(List<string> AddInformation)
        {
            // 1. 为ToolBase表增加条目，占10个数据
            int[] idInformation = RigidAddItem("dbo.ToolBase", AddInformation.GetRange(0, 10));
            int givenLineNumber = idInformation[0];
            int givenToolID = idInformation[1];

            if (givenToolID < 0)
            {
                return -1;
            }

            // 2. 为ToolPosition表增加条目，占12个数据
            List<string> hangedInformation = new List<string>(AddInformation.GetRange(10, 6));
            hangedInformation.Insert(0, "True");
            List<string> unHangedInformation = new List<string>(AddInformation.GetRange(16, 6));
            unHangedInformation.Insert(0, "False");

            bool addResult = SoftAddItem("dbo.ToolPosition", givenToolID, hangedInformation);
            addResult = SoftAddItem("dbo.ToolPosition", givenToolID, unHangedInformation);

            if (!addResult)
            {
                return -1;
            }

            // 3. 删除工具号对应的索引表内容
            if (!DeleteItemByLineNumber("dbo.NumIDRecord", givenLineNumber))
            {
                return -1;
            }

            // 4. 判断索引表是否为空，为空则增加下一个条目，不为空则更新Id值
            int indexLength = AskCurrentItemLength("dbo.NumIDRecord");
            if (indexLength < 0)
            {
                return -1;
            }
            else if (indexLength < 1)
            {
                int toolID = AskCurrentItemLength("dbo.ToolBase") + 1;
                if (toolID < 0)
                {
                    return -1;
                }

                if (!SoftAddItem("dbo.NumIDRecord", toolID, new List<string>()))
                {
                    return -1;
                }
            }
            else
            {
                ReSortLineNumber("dbo.NumIDRecord");
            }

            return givenToolID;
        }

        /// <summary>
        /// 删除已有工具
        /// </summary>
        /// <param name="ToolNumber">要删除的工具号</param>
        /// <returns>返回删除的结果</returns>
        public virtual bool DeleteExistTool(int ToolNumber)
        {
            // 1. 删除ToolBase表中的工具信息
            if (!DeleteItemByToolNumber("dbo.ToolBase", ToolNumber))
            {
                return false;
            }

            // 2. 重排ToolBase表的Id值
            ReSortLineNumber("dbo.ToolBase");

            // 3. 删除ToolPosition表中的工具信息
            if (!DeleteItemByToolNumber("dbo.ToolPosition", ToolNumber))
            {
                return false;
            }

            // 4. 重排ToolPosition表的Id值
            ReSortLineNumber("dbo.ToolPosition");

            // 5. 增加被删除的工具号到工具号索引表
            if (!SoftAddItem("dbo.NumIDRecord", ToolNumber, new List<string>()))
            {
                return false;
            }

            object[] getObject = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceX WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
            if (getObject.Length < 1) // 没有工具号对应的力信息
            {
                return true;
            }

            // 6. 删除六维力表中的工具力信号信息
            if (!DeleteItemByToolNumber("dbo.FlangeForceX", ToolNumber) ||
                !DeleteItemByToolNumber("dbo.FlangeForceY", ToolNumber) ||
                !DeleteItemByToolNumber("dbo.FlangeForceZ", ToolNumber) ||
                !DeleteItemByToolNumber("dbo.FlangeTorqueX", ToolNumber) ||
                !DeleteItemByToolNumber("dbo.FlangeTorqueY", ToolNumber) ||
                !DeleteItemByToolNumber("dbo.FlangeTorqueZ", ToolNumber))
            {
                return false;
            }

            // 7. 重排六维力表的Id值
            ReSortLineNumber("dbo.FlangeForceX");
            ReSortLineNumber("dbo.FlangeForceY");
            ReSortLineNumber("dbo.FlangeForceZ");
            ReSortLineNumber("dbo.FlangeTorqueX");
            ReSortLineNumber("dbo.FlangeTorqueY");
            ReSortLineNumber("dbo.FlangeTorqueZ");

            return true;
        }

        /// <summary>
        /// 更新工具参数
        /// </summary>
        /// <param name="ToolNumber">要更新的工具号</param>
        /// <param name="UpdateContent">要更新的内容键值对</param>
        /// <returns>更新的结果</returns>
        public virtual bool UpdateExistTool(int ToolNumber, Dictionary<string, string> UpdateContent)
        {
            foreach (KeyValuePair<string, string> item in UpdateContent)
            {
                switch (item.Key)
                {
                    case "TcpX":
                    case "TcpY":
                    case "TcpZ":
                    case "TcpRX":
                    case "TcpRY":
                    case "TcpRZ":
                    case "Gravity":
                    case "GravityX":
                    case "GravityY":
                    case "GravityZ":
                        if (!UpdateOneField("dbo.ToolBase", ToolNumber, item.Key, item.Value))
                        {
                            return false;
                        }
                        break;
                    case "Joint1Hanged":
                    case "Joint2Hanged":
                    case "Joint3Hanged":
                    case "Joint4Hanged":
                    case "Joint5Hanged":
                    case "Joint6Hanged":
                        if (!UpdateOneFieldForMultiplyRows("dbo.ToolPosition", ToolNumber, item.Key.Remove(6), item.Value, "IfHanged", "True"))
                        {
                            return false;
                        }
                        break;
                    case "Joint1UnHanged":
                    case "Joint2UnHanged":
                    case "Joint3UnHanged":
                    case "Joint4UnHanged":
                    case "Joint5UnHanged":
                    case "Joint6UnHanged":
                        if (!UpdateOneFieldForMultiplyRows("dbo.ToolPosition", ToolNumber, item.Key.Remove(6), item.Value, "IfHanged", "False"))
                        {
                            return false;
                        }
                        break;
                    default:
                        return false;
                }
            }

            return true;
        }

        /// <summary>
        /// 刷新相应工具号工具的力信息
        /// </summary>
        /// <param name="ToolNumber">要刷新的工具号</param>
        /// <param name="ForceContent">要使用的力信息</param>
        /// <param name="ContentLength">力信号的单元个数</param>
        /// <returns>刷新的结果</returns>
        public virtual bool RefreshForceTabels(int ToolNumber, double[,] ForceContent, int ContentLength)
        {
            List<string> fx = new List<string>(200);
            List<string> fy = new List<string>(200);
            List<string> fz = new List<string>(200);
            List<string> tx = new List<string>(200);
            List<string> ty = new List<string>(200);
            List<string> tz = new List<string>(200);

            for (int i = 0; i < ContentLength; i++)
            {
                fx.Add(ForceContent[0, i].ToString("0.00"));
                fy.Add(ForceContent[1, i].ToString("0.00"));
                fz.Add(ForceContent[2, i].ToString("0.00"));
                tx.Add(ForceContent[3, i].ToString("0.000"));
                ty.Add(ForceContent[4, i].ToString("0.000"));
                tz.Add(ForceContent[5, i].ToString("0.000"));
            }

            object[] getObject = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceX WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
            if (getObject.Length >= 1) // 有工具号对应的力信息，要先删除条目
            {
                // 删除相关条目
                if (!DeleteItemByToolNumber("dbo.FlangeForceX", ToolNumber) ||
                    !DeleteItemByToolNumber("dbo.FlangeForceY", ToolNumber) ||
                    !DeleteItemByToolNumber("dbo.FlangeForceZ", ToolNumber) ||
                    !DeleteItemByToolNumber("dbo.FlangeTorqueX", ToolNumber) ||
                    !DeleteItemByToolNumber("dbo.FlangeTorqueY", ToolNumber) ||
                    !DeleteItemByToolNumber("dbo.FlangeTorqueZ", ToolNumber))
                {
                    return false;
                }

                // 重排Id
                ReSortLineNumber("dbo.FlangeForceX");
                ReSortLineNumber("dbo.FlangeForceY");
                ReSortLineNumber("dbo.FlangeForceZ");
                ReSortLineNumber("dbo.FlangeTorqueX");
                ReSortLineNumber("dbo.FlangeTorqueY");
                ReSortLineNumber("dbo.FlangeTorqueZ");
            }

            // 增加条目
            bool end = SoftAddItem("dbo.FlangeForceX", ToolNumber, fx);
            end = SoftAddItem("dbo.FlangeForceY", ToolNumber, fy);
            end = SoftAddItem("dbo.FlangeForceZ", ToolNumber, fz);
            end = SoftAddItem("dbo.FlangeTorqueX", ToolNumber, tx);
            end = SoftAddItem("dbo.FlangeTorqueY", ToolNumber, ty);
            end = SoftAddItem("dbo.FlangeTorqueZ", ToolNumber, tz);

            if (!end)
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// 刷新相应工具号工具的完整力信息
        /// </summary>
        /// <param name="ToolNumber">要刷新的工具号</param>
        /// <param name="ForceContent">要使用的力信息</param>
        /// <param name="TableNum">完整力信号的表格标号</param>
        /// <returns>刷新的结果</returns>
        public virtual bool RefreshEntireForceTabels(int ToolNumber, double[,] ForceContent, int TableNum)
        {
            List<string> fx = new List<string>(600);
            List<string> fy = new List<string>(600);
            List<string> fz = new List<string>(600);
            List<string> tx = new List<string>(600);
            List<string> ty = new List<string>(600);
            List<string> tz = new List<string>(600);

            for (int i = 0; i < 540; i++)
            {
                fx.Add(ForceContent[0, i].ToString("0.00"));
                fy.Add(ForceContent[1, i].ToString("0.00"));
                fz.Add(ForceContent[2, i].ToString("0.00"));
                tx.Add(ForceContent[3, i].ToString("0.000"));
                ty.Add(ForceContent[4, i].ToString("0.000"));
                tz.Add(ForceContent[5, i].ToString("0.000"));
            }

            string tableName = "";
            switch (TableNum)
            {
                case 1: tableName = "dbo.EntireFlangeForce0001To0540"; break;
                case 2: tableName = "dbo.EntireFlangeForce0541To1080"; break;
                case 3: tableName = "dbo.EntireFlangeForce1081To1620"; break;
                case 4: tableName = "dbo.EntireFlangeForce1621To2160"; break;
                case 5: tableName = "dbo.EntireFlangeForce2161To2700"; break;
                case 6: tableName = "dbo.EntireFlangeForce2701To3240"; break;
                case 7: tableName = "dbo.EntireFlangeForce3241To3780"; break;
                case 8: tableName = "dbo.EntireFlangeForce3781To4320"; break;
                case 9: tableName = "dbo.EntireFlangeForce4321To4860"; break;
                case 10: tableName = "dbo.EntireFlangeForce4861To5400"; break;
                case 11: tableName = "dbo.EntireFlangeForce5401To5940"; break;
                case 12: tableName = "dbo.EntireFlangeForce5941To6480"; break;
                case 13: tableName = "dbo.EntireFlangeForce6481To7020"; break;
                case 14: tableName = "dbo.EntireFlangeForce7021To7560"; break;
                case 15: tableName = "dbo.EntireFlangeForce7561To8100"; break;
                default: return false;
            }

            // 查找条目是否存在
            object[] getObject = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
            if (getObject.Length >= 1) // 有工具号对应的力信息，要先删除条目
            {
                // 删除相关条目
                if (!DeleteItemByToolNumber(tableName, ToolNumber))
                    return false;
            }

            // 增加条目
            int usedTableRowNumber = AskCurrentItemLength(tableName) + 1;

            bool endResult = true;
            for (int i = 0; i < 6; i++)
            {
                string content = "(\'" + (usedTableRowNumber + i).ToString("0") + "\'";
                content += ", \'" + ToolNumber.ToString("0") + "\', \'" + i.ToString("0") + "\'";

                switch (i)
                {
                    case 0: foreach (string item in fx) content += ", \'" + item + "\'"; break;
                    case 1: foreach (string item in fy) content += ", \'" + item + "\'"; break;
                    case 2: foreach (string item in fz) content += ", \'" + item + "\'"; break;
                    case 3: foreach (string item in tx) content += ", \'" + item + "\'"; break;
                    case 4: foreach (string item in ty) content += ", \'" + item + "\'"; break;
                    case 5: foreach (string item in tz) content += ", \'" + item + "\'"; break;
                    default: return false;
                }
                content += ")";
                string addingStr = "INSERT INTO " + tableName + " VALUES " + content;
                endResult = SendNoReplyCommandToDataBase(addingStr);

                if (!endResult) return false;
            }
            return true;
        }

        /// <summary>
        /// 查询相应工具的基本工具信息
        /// </summary>
        /// <param name="ToolNumber">工具号</param>
        /// <returns>返回基本工具信息，-1无效</returns>
        public virtual double[] SearchToolBaseInformation(int ToolNumber)
        {
            object[] recieveDatas = SendWithReplyCommandToDataBase("SELECT * FROM dbo.ToolBase WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
            if (recieveDatas.Length < 1)
            {
                return new double[] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
            }

            List<object> baseInformation = new List<object>(recieveDatas);
            List<double> backDatas = new List<double>(10);
            foreach (object item in baseInformation.GetRange(2, 10))
            {
                backDatas.Add(double.Parse(((decimal)item).ToString()));
            }

            return backDatas.ToArray();
        }

        /// <summary>
        /// 查询相应工具的工具位置信息
        /// </summary>
        /// <param name="ToolNumber">工具号</param>
        /// <returns>返回工具位置信息，-1无效</returns>
        public virtual double[] SearchToolPositionInformation(int ToolNumber)
        {
            object[] recieveDatas = SendWithReplyCommandToDataBase("SELECT * FROM dbo.ToolPosition WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
            if (recieveDatas.Length < 1)
            {
                return new double[] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
            }

            List<object> positionInformation = new List<object>(recieveDatas);
            List<double> backDatas = new List<double>(10);
            backDatas.Add((bool)positionInformation[2] ? 1.0 : 0.0);
            foreach (object item in positionInformation.GetRange(3, 6))
            {
                backDatas.Add(double.Parse(((decimal)item).ToString()));
            }
            backDatas.Add((bool)positionInformation[11] ? 1.0 : 0.0);
            foreach (object item in positionInformation.GetRange(12, 6))
            {
                backDatas.Add(double.Parse(((decimal)item).ToString()));
            }
            return backDatas.ToArray();
        }

        /// <summary>
        /// 查询相应工具的工具力信息
        /// </summary>
        /// <param name="ToolNumber">工具号</param>
        /// <param name="ifWhole">是否返回完整信息</param>
        /// <returns>返回工具力信息，-1无效</returns>
        public virtual double[,] SearchToolForceInformation(int ToolNumber, bool ifWhole = false)
        {
            if (ifWhole)
            {
                object[] getObject = SendWithReplyCommandToDataBase("SELECT * FROM dbo.EntireFlangeForce0001To0540 WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'0\'");
                if (getObject.Length < 1) // 没有工具号对应的力信息
                {
                    return new double[,] { { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 } };
                }

                double[,] forceInformation = new double[6, 180 * 3 * 15];
                for (int i = 1; i < 16; i++)
                {
                    string tableName = "";
                    switch (i)
                    {
                        case 1: tableName = "dbo.EntireFlangeForce0001To0540"; break;
                        case 2: tableName = "dbo.EntireFlangeForce0541To1080"; break;
                        case 3: tableName = "dbo.EntireFlangeForce1081To1620"; break;
                        case 4: tableName = "dbo.EntireFlangeForce1621To2160"; break;
                        case 5: tableName = "dbo.EntireFlangeForce2161To2700"; break;
                        case 6: tableName = "dbo.EntireFlangeForce2701To3240"; break;
                        case 7: tableName = "dbo.EntireFlangeForce3241To3780"; break;
                        case 8: tableName = "dbo.EntireFlangeForce3781To4320"; break;
                        case 9: tableName = "dbo.EntireFlangeForce4321To4860"; break;
                        case 10: tableName = "dbo.EntireFlangeForce4861To5400"; break;
                        case 11: tableName = "dbo.EntireFlangeForce5401To5940"; break;
                        case 12: tableName = "dbo.EntireFlangeForce5941To6480"; break;
                        case 13: tableName = "dbo.EntireFlangeForce6481To7020"; break;
                        case 14: tableName = "dbo.EntireFlangeForce7021To7560"; break;
                        case 15: tableName = "dbo.EntireFlangeForce7561To8100"; break;
                        default:
                            return new double[,] { { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 } };
                    }

                    object[] recievedFx = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'0\'");
                    object[] recievedFy = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'1\'");
                    object[] recievedFz = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'2\'");
                    object[] recievedTx = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'3\'");
                    object[] recievedTy = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'4\'");
                    object[] recievedTz = SendWithReplyCommandToDataBase("SELECT * FROM " + tableName + " WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\' AND Axis=\'5\'");

                    if (recievedFx.Length < 1 || recievedFy.Length < 1 || recievedFz.Length < 1 || recievedTx.Length < 1 || recievedTy.Length < 1 || recievedTz.Length < 1)
                    {
                        return new double[,] { { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 } };
                    }

                    for (int j = 0; j < 180 * 3; j++)
                    {
                        forceInformation[0, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedFx[j + 3]).ToString());
                        forceInformation[1, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedFy[j + 3]).ToString());
                        forceInformation[2, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedFz[j + 3]).ToString());
                        forceInformation[3, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedTx[j + 3]).ToString());
                        forceInformation[4, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedTy[j + 3]).ToString());
                        forceInformation[5, 180 * 3 * (i - 1) + j] = double.Parse(((decimal)recievedTz[j + 3]).ToString());
                    }
                }

                return forceInformation;
            }
            else
            {
                object[] getObject = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceX WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                if (getObject.Length < 1) // 没有工具号对应的力信息
                {
                    return new double[,] { { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 } };
                }

                object[] recievedFx = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceX WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                object[] recievedFy = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceY WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                object[] recievedFz = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeForceZ WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                object[] recievedTx = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeTorqueX WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                object[] recievedTy = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeTorqueY WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");
                object[] recievedTz = SendWithReplyCommandToDataBase("SELECT * FROM dbo.FlangeTorqueZ WHERE ToolNum=\'" + ToolNumber.ToString("0") + "\'");

                if (recievedFx.Length < 1 || recievedFy.Length < 1 || recievedFz.Length < 1 || recievedTx.Length < 1 || recievedTy.Length < 1 || recievedTz.Length < 1)
                {
                    return new double[,] { { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 }, { -1.0 } };
                }

                double[,] forceInformation = new double[6, recievedFx.Length - 2];

                for (int i = 0; i < recievedFx.Length - 2; i++)
                {
                    forceInformation[0, i] = double.Parse(((decimal)recievedFx[i + 2]).ToString());
                    forceInformation[1, i] = double.Parse(((decimal)recievedFy[i + 2]).ToString());
                    forceInformation[2, i] = double.Parse(((decimal)recievedFz[i + 2]).ToString());
                    forceInformation[3, i] = double.Parse(((decimal)recievedTx[i + 2]).ToString());
                    forceInformation[4, i] = double.Parse(((decimal)recievedTy[i + 2]).ToString());
                    forceInformation[5, i] = double.Parse(((decimal)recievedTz[i + 2]).ToString());
                }

                return forceInformation;
            }




           
        }
        #endregion
    }
}
