using System;
using System.Runtime.InteropServices;

namespace SensorAgent
{
    class ImportGyroFunctions
    {
        [DllImport(@".\GyroSDK.dll", EntryPoint = "ConnectToSensor",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool ConnectToSensor();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "DisconnectFromSensor",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool DisconnectFromSensor();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "AskDeviceExist",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool AskDeviceExist();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "RefreshDeviceInfo",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool RefreshDeviceInfo();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "GetInfoX",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoX();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "GetInfoY",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoY();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "GetInfoZ",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoZ();

        [DllImport(@".\GyroSDK.dll", EntryPoint = "GetInfoW",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoW();

        public static double[] GetInfo()
        {
            bool refreshResult = RefreshDeviceInfo();
            if (!refreshResult) return null;
            double[] infos = new double[4];
            infos[0] = GetInfoX() / 10000.0;
            infos[1] = GetInfoY() / 10000.0;
            infos[2] = GetInfoZ() / 10000.0;
            infos[3] = GetInfoW() / 10000.0;
            return infos;
        }
    }
}
