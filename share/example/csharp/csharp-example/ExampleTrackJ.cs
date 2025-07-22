using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using System.Threading;

namespace csharp_example
{

    class ExampleTrackJ
    {
        const int RSERR_SUCC = 0;

        static UInt16 rshd = 0xffff;
        // Robot IP address
        const string robot_ip = "192.168.204.151";
        // Robot port number
        const int server_port = 30004;
        // M_PI
        const double M_PI = 3.14159265358979323846;

        // Blocking function: The program continues when the robot reaches the target waypoint
        static int waitArrival(IntPtr robot_interface)
        {
            const int max_retry_count = 5;
            int cnt = 0;

            // API call: Get the current motion command ID
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            int exec_id = cSharpBinging_MotionControl.getExecId(motion_control);

            // Wait for the robot to start moving
            while (exec_id == -1)
            {
                if (cnt++ > max_retry_count)
                {
                    return -1;
                }
                Thread.Sleep(50);
                exec_id = cSharpBinging_MotionControl.getExecId(motion_control);
            }

            // Wait for the robot to finish moving
            while (cSharpBinging_MotionControl.getExecId(motion_control) != -1)
            {
                Thread.Sleep(50);
            }

            return 0;
        }

        // Trajectory data reading class
        class TrajectoryIo
        {
            private StreamReader input_file;

            // Constructor, accepts the filename to open as a parameter
            public TrajectoryIo(string filename)
            {
                try
                {
                    input_file = new StreamReader(filename);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Unable to open trajectory file. Please check if the input file path is correct. Exception info: {ex.Message}");
                }
            }

            // Check if the file was successfully opened
            public bool Open()
            {
                return input_file != null;
            }

            // Parse trajectory data from the file
            public List<List<double>> Parse()
            {
                List<List<double>> res = new List<List<double>>();
                string tmp;
                int linenum = 1;
                while ((tmp = input_file.ReadLine()) != null)
                {
                    try
                    {
                        List<double> q = Split(tmp, ",");
                        res.Add(q);
                    }
                    catch (Exception p)
                    {
                        Console.WriteLine($"Line: {linenum} \"{p.Message}\" is not a number of double");
                        break;
                    }
                    linenum++;
                }
                return res;
            }

            // Split string and convert to double type
            private List<double> Split(string str, string delim)
            {
                List<double> res = new List<double>();
                if (string.IsNullOrEmpty(str))
                {
                    return res;
                }

                string[] parts = str.Split(delim.ToCharArray(), StringSplitOptions.RemoveEmptyEntries);
                foreach (string part in parts)
                {
                    if (double.TryParse(part, out double v))
                    {
                        res.Add(v);
                    }
                    else
                    {
                        throw new Exception(part);
                    }
                }
                return res;
            }
        }

        static int exampleTrackJoint(IntPtr cli)
        {
            // Read trajectory file
            string filename = "../trajs/record6.offt";
            TrajectoryIo input = new TrajectoryIo(filename);

            // Try to open trajectory file, return directly if unable to open
            if (!input.Open())
            {
                return 0;
            }

            // Parse trajectory data
            List<List<double>> traj = input.Parse();

            // Check if there are waypoints in the trajectory file
            if (traj.Count == 0)
            {
                Console.WriteLine("Number of waypoints in trajectory file is 0.");
                return 0;
            }

            // API call: Get robot names
            IntPtr[] robot_names = new IntPtr[10];
            for (int i = 0; i < 10; i++)
            {
                robot_names[i] = Marshal.AllocHGlobal(100); // Allocate 100 bytes for string storage (considering '\0' ending)
            }

            int num = cSharpBinding_RPC.rpc_getRobotNames(cli, robot_names);
            if (num <= 0)
            {
                for (int i = 0; i < 10; i++)
                {
                    Marshal.FreeHGlobal(robot_names[i]); // Free allocated memory
                }
                return -1;
            }
            string robot_name = Marshal.PtrToStringAnsi(robot_names[0]);
            for (int i = 0; i < 10; i++)
            {
                Marshal.FreeHGlobal(robot_names[i]); // Free allocated memory
            }
            if (robot_name == "")
            {
                return -1;
            }

            IntPtr robot_interface = cSharpBinding_RPC.rpc_getRobotInterface(cli, robot_name);

            // API call: Set robot speed fraction
            IntPtr motion_control = cSharpBinging_RobotInterface.robot_getMotionControl(robot_interface);
            cSharpBinging_MotionControl.setSpeedFraction(motion_control, 0.3);

            // API call: Move joint to the first point in the trajectory to avoid large overshoot
            cSharpBinging_MotionControl.moveJoint(motion_control, traj[0].ToArray(), 80 * (M_PI / 180),
                                                          60 * (M_PI / 180), 0, 0);

            // Blocking
            int ret = waitArrival(robot_interface);
            if (ret == 0)
            {
                Console.WriteLine("Joint moved to the first waypoint in the trajectory file successfully");
            }
            else
            {
                Console.WriteLine("Joint failed to move to the first waypoint in the trajectory file");
            }

            for (int i = 1; i < traj.Count; i++)
            {
                int traj_queue_size = cSharpBinging_MotionControl.getTrajectoryQueueSize(motion_control);
                Console.WriteLine($"traj_queue_size: {traj_queue_size}");
                while (traj_queue_size > 8)
                {
                    traj_queue_size = cSharpBinging_MotionControl.getTrajectoryQueueSize(motion_control);
                    Thread.Sleep(1);
                }

                cSharpBinging_MotionControl.moveJoint(motion_control, traj[i].ToArray(), 0.01, 0.5, 1, 0);

            }

            // Wait for motion to finish
            IntPtr robot_state = cSharpBinging_RobotInterface.robot_getRobotState(robot_interface);
            while (!cSharpBinging_RobotState.isSteady(robot_state))
            {
                Thread.Sleep(5);
            }

            cSharpBinging_MotionControl.stopJoint(motion_control, 1);

            Console.WriteLine("trackJoint motion finished");

            return 0;
        }


        static void Main_TrackJ()
        {
#if NETCOREAPP
            // Get maximum real-time priority
            int sched_max = GetMaxPriority(); 

            // Set real-time scheduling policy and priority
            SchedParam sParam = new SchedParam();
            sParam.Priority = sched_max;
            SetScheduler(0, SchedType.FIFO, sParam);
            int i_schedFlag = GetScheduler(0);
            Console.WriteLine($"Set scheduling policy = [{i_schedFlag}]");

            // Bind CPU
            CpuSet cpuset = new CpuSet();
            CpuSet.Clear(cpuset);
            CpuSet.Set(1, cpuset);
            SetAffinity(0, cpuset);
#endif

            IntPtr rpc_client = cSharpBinding_RPC.rpc_create_client(0);
            Console.Out.WriteLine("rpc_create_client ret={0}", rpc_client);
            if (rpc_client == IntPtr.Zero)
            {
                Console.Error.WriteLine("rpc_create_client failed!");
            }

            cSharpBinding_RPC.rpc_connect(rpc_client, robot_ip, server_port);
            cSharpBinding_RPC.rpc_setRequestTimeout(rpc_client, 1000);
            cSharpBinding_RPC.rpc_login(rpc_client, "aubo", "123456");

            exampleTrackJoint(rpc_client);

            cSharpBinding_RPC.rpc_logout(rpc_client);
            cSharpBinding_RPC.rpc_disconnect(rpc_client);
        }



    }
}
