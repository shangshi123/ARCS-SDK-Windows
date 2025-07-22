using System;
using System.Threading;
using System.Runtime.InteropServices;

namespace csharp_example
{

    class ExampleStartup
    {

        const int RSERR_SUCC = 0;

        static UInt16 rshd = 0xffff;
        // Robot IP address
        const string robot_ip = "192.168.204.151";
        // Robot port number
        const int server_port = 30004;


        // Wait for the robot to enter the target mode
        static void WaitForRobotMode(IntPtr robot_state, cSharpBinging_TypeDef.RobotModeType target_mode)
        {
            // API call: Get the current mode of the robot
            cSharpBinging_TypeDef.RobotModeType current_mode = cSharpBinging_RobotState.getRobotModeType(robot_state);

            while (current_mode != target_mode)
            {
                Console.WriteLine($"Current robot mode: {current_mode}");
                Thread.Sleep(1000);
                current_mode = cSharpBinging_RobotState.getRobotModeType(robot_state);
            }
        }

        static int exampleStartup(IntPtr cli)
        {
            IntPtr[] robot_names = new IntPtr[10];
            for (int i = 0; i < 10; i++)
            {
                robot_names[i] = Marshal.AllocHGlobal(100); // Allocate 100 bytes of memory for storing strings (considering '\0' ending)
            }

            int num = cSharpBinding_RPC.rpc_getRobotNames(cli, robot_names);
            if (num <= 0)
            {
                for (int i = 0; i < 10; i++)
                {
                    Marshal.FreeHGlobal(robot_names[i]); // Release allocated memory
                }
                return -1;
            }
            string robot_name = Marshal.PtrToStringAnsi(robot_names[0]);
            for (int i = 0; i < 10; i++)
            {
                Marshal.FreeHGlobal(robot_names[i]); // Release allocated memory
            }
            if (robot_name == "") {
                return -1;
            }

            IntPtr robot_interface = cSharpBinding_RPC.rpc_getRobotInterface(cli, robot_name);

            // API call: Set payload
            double mass = 0.0; // Example mass value
            double[] cog =  { 0.0, 0.0, 0.0 }; // Example center of gravity coordinates
            double[] aom =  { 0.0, 0.0, 0.0 }; // Example additional mass moment
            double[] inertia = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Example inertia tensor

            IntPtr robot_config = cSharpBinging_RobotInterface.robot_getRobotConfig(robot_interface);

           cSharpBinging_RobotConfig.setPayload(robot_config, mass, cog, aom, inertia);

            IntPtr robot_state = cSharpBinging_RobotInterface.robot_getRobotState(robot_interface);
            // API call: Get the current mode of the robot
            cSharpBinging_TypeDef.RobotModeType robot_mode = cSharpBinging_RobotState.getRobotModeType(robot_state);

            if (robot_mode == cSharpBinging_TypeDef.RobotModeType.Running)
            {
                Console.WriteLine("Robot brake released, in running mode");
                return -1;
            }

            IntPtr robot_manage = cSharpBinging_RobotInterface.robot_getRobotManage(robot_interface);
            // API call: Robot initiates power-on request
            cSharpBinging_RobotManage.poweron(robot_manage);

           
            // Wait for the robot to enter idle mode
            WaitForRobotMode(robot_state, cSharpBinging_TypeDef.RobotModeType.Idle);

            robot_mode = cSharpBinging_RobotState.getRobotModeType(robot_state);
            Console.WriteLine($"Robot powered on successfully, current mode:{0}", robot_mode);

            // API call: Robot initiates brake release request
            cSharpBinging_RobotManage.startup(robot_manage);

            // Wait for the robot to enter running mode
            WaitForRobotMode(robot_state, cSharpBinging_TypeDef.RobotModeType.Running);

            robot_mode = cSharpBinging_RobotState.getRobotModeType(robot_state);
            Console.WriteLine($"Robot brake released successfully, current mode: {0}", robot_mode);

            return 0;
        }

        static void Main(string[] args)
        {
            cSharpBinding_RPC csharpbingding_rpc = new cSharpBinding_RPC();
            // Initialize robot control library
            IntPtr rpc_client = cSharpBinding_RPC.rpc_create_client(0);
            Console.Out.WriteLine("rpc_create_client ret={0}", rpc_client);
            if (rpc_client == IntPtr.Zero)
            {
                Console.Error.WriteLine("rpc_create_client failed!");
            }

            cSharpBinding_RPC.rpc_connect(rpc_client, robot_ip, server_port);
            cSharpBinding_RPC.rpc_setRequestTimeout(rpc_client, 1000);
            cSharpBinding_RPC.rpc_login(rpc_client, "aubo", "123456");

            exampleStartup(rpc_client);

            cSharpBinding_RPC.rpc_logout(rpc_client);
            cSharpBinding_RPC.rpc_disconnect(rpc_client);
        }

    }
}

