using System.Collections.Generic;

namespace SQLServerConnection
{
    /// <summary>
    /// SQL Server数据交换类
    /// </summary>
    public class SQLServerExchanger : SQLServerExchangeBase
    {
        #region 方法
        /// <summary>
        /// 查询指定表格的条目数
        /// </summary>
        /// <param name="TableStr">指定表格的名称</param>
        /// <returns>条目数，-1为无效</returns>
        protected int AskCurrentItemLength(string TableStr)
        {
            string checkingStr = "SELECT COUNT(*) FROM " + TableStr;
            object[] checkResult = SendWithReplyCommandToDataBase(checkingStr);

            if (checkResult.Length < 1)
            {
                return -1;
            }
            else
            {
                return (int)checkResult[0];
            }
        }

        /// <summary>
        /// 查询可用的最小工具编号和其行号
        /// </summary>
        /// <returns>最小可用工具编号和其行号，-1为超界</returns>
        protected int[] AskPosibleToolNumAndLineNumber()
        {
            string checkingStr = "SELECT * FROM NumIDRecord WHERE PossibleToolNum IN (SELECT MIN(PossibleToolNum) FROM NumIDRecord)";
            object[] checkResult = SendWithReplyCommandToDataBase(checkingStr);
           
            if (checkResult.Length < 1)
            {
                return new int[] { -1, -1 };
            }
            else
            {
                return new int[] { (int)checkResult[0], (int)checkResult[1] };
            }
        }

        /// <summary>
        /// 有工具号限制得增加条目
        /// </summary>
        /// <param name="TableStr">要增加的表格</param>
        /// <param name="AddContent">要增加的内容</param>
        /// <returns>增加条目所用的工具号及其存储的行号，-1为失败</returns>
        protected virtual int[] RigidAddItem(string TableStr, List<string> AddContent)
        {
            int usedTableRowNumber = AskCurrentItemLength(TableStr) + 1;
            int[] usedIDInformation = AskPosibleToolNumAndLineNumber();

            string content = "(\'" + usedTableRowNumber.ToString("0") + "\', \'" + usedIDInformation[1].ToString("0") + "\'";
            foreach (string item in AddContent)
            {
                content += ", \'" + item + "\'";
            }
            content += ")";

            string addingStr = "INSERT INTO " + TableStr + " VALUES " + content;
            bool operateResult = SendNoReplyCommandToDataBase(addingStr);

            if (operateResult)
            {
                return new int[] { usedIDInformation[0], usedIDInformation[1] };
            }
            else
            {
                return new int[] { -1, -1 };
            }
        }

        /// <summary>
        /// 无额外限制得增加条目
        /// </summary>
        /// <param name="TableStr">要增加的表格</param>
        /// <param name="ToolNum">要增加的工具号</param>
        /// <param name="AddContent">要增加的内容</param>
        /// <returns>增加结果</returns>
        protected virtual bool SoftAddItem(string TableStr, int ToolNum, List<string> AddContent)
        {
            int usedTableRowNumber = AskCurrentItemLength(TableStr) + 1;

            string content = "(\'" + usedTableRowNumber.ToString("0") + "\'";
            content += ", \'" + ToolNum.ToString("0") + "\'";
            foreach (string item in AddContent)
            {
                content += ", \'" + item + "\'";
            }
            content += ")";

            string addingStr = "INSERT INTO " + TableStr + " VALUES " + content;
            return SendNoReplyCommandToDataBase(addingStr);
        }

        /// <summary>
        /// 通过行号删除指定表中的指定行条目
        /// </summary>
        /// <param name="TableStr">要删除的表</param>
        /// <param name="LineNumber">要删除的行号</param>
        /// <returns>删除的结果</returns>
        protected virtual bool DeleteItemByLineNumber(string TableStr, int LineNumber)
        {
            string deleteStr = "DELETE FROM " + TableStr + " WHERE Id = \'" + LineNumber.ToString("0") + "\' ";
            return SendNoReplyCommandToDataBase(deleteStr);
        }

        /// <summary>
        /// 通过工具号删除指定表中的指定行条目
        /// </summary>
        /// <param name="TableStr">要删除的表</param>
        /// <param name="ToolNumber">要删除的工具号</param>
        /// <returns>删除的结果</returns>
        protected virtual bool DeleteItemByToolNumber(string TableStr, int ToolNumber)
        {
            string deleteStr = "DELETE FROM " + TableStr + " WHERE ToolNum = \'" + ToolNumber.ToString("0") + "\' ";
            return SendNoReplyCommandToDataBase(deleteStr);
        }

        /// <summary>
        /// 重排指定表的Id号
        /// </summary>
        /// <param name="TableStr">要重排的表</param>
        protected virtual void ReSortLineNumber(string TableStr)
        {
            string reSortStr = "UPDATE " + TableStr + " SET Id=SortTable.NewId " +
                                               "FROM(SELECT Id,ROW_NUMBER() " +
                                               "OVER(ORDER BY Id ASC) AS NewId " +
                                               "FROM " + TableStr + ") AS SortTable " +
                                         "WHERE SortTable.Id=" + TableStr + ".Id";

            SendNoReplyCommandToDataBase(reSortStr);
        }

        /// <summary>
        /// 更新对应工具的单个字段
        /// </summary>
        /// <param name="TableStr">要更新的表</param>
        /// <param name="ToolNumebr">要更新的工具号</param>
        /// <param name="FieldStr">要更新的字段</param>
        /// <param name="FieldValue">要更新的数据</param>
        /// <returns>更新的结果</returns>
        protected virtual bool UpdateOneField(string TableStr, int ToolNumebr, string FieldStr, string FieldValue)
        {
            string updateStr = "UPDATE " + TableStr + " SET " + FieldStr + " = \'" + FieldValue + "\' WHERE ToolNum = \'" + ToolNumebr.ToString("0") + "\'";
            return SendNoReplyCommandToDataBase(updateStr);
        }

        /// <summary>
        /// 更新对应工具的多行单字段
        /// </summary>
        /// <param name="TableStr">要更新的表</param>
        /// <param name="ToolNumebr">要更新的工具号</param>
        /// <param name="FieldStr">要更新的字段</param>
        /// <param name="FieldValue">要更新的数据</param>
        /// <param name="RowDivider">区分不同行的特征</param>
        /// <param name="RowDividerValue">区分不同行的特征值</param>
        /// <returns>更新的结果</returns>
        protected virtual bool UpdateOneFieldForMultiplyRows(string TableStr, int ToolNumebr, string FieldStr, string FieldValue, string RowDivider, string RowDividerValue)
        {
            string updateStr = "UPDATE " + TableStr + " SET " + FieldStr + " = \'" + FieldValue + "\' WHERE ToolNum = \'" + ToolNumebr.ToString("0") + "\' and " + RowDivider + " = \'" + RowDividerValue + "\'";
            return SendNoReplyCommandToDataBase(updateStr);
        }
        #endregion
    }
}
