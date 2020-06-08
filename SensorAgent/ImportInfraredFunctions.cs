using System;
using System.Runtime.InteropServices;

namespace SensorAgent
{
    class ImportInfraredFunctions
    {
        [DllImport(@".\InfraredSDK.dll", EntryPoint = "ConnectToSensor",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool ConnectToSensor();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "DisconnectFromSensor",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool DisconnectFromSensor();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "AskDeviceExist",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        public static extern bool AskDeviceExist();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "GetInfoX",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoX();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "GetInfoY",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoY();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "GetInfoL",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoL();

        [DllImport(@".\InfraredSDK.dll", EntryPoint = "GetInfoW",
         CharSet = CharSet.Unicode, CallingConvention = CallingConvention.Cdecl)]
        private static extern Int32 GetInfoW();

        public static double[] GetInfo()
        {
            if (!AskDeviceExist()) return null;
            double[] infos = new double[4];
            infos[1] = 380.0 - GetInfoX() * 380.0 / 0x7fff;
            infos[0] = 302.0 - GetInfoY() * 302.0 / 0x7fff;
            infos[2] = GetInfoL() * 0.14;
            infos[3] = GetInfoW() * 0.14;
            return infos;
        }
    }
}
