﻿using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Net.Sockets;
using System.Net;
using System.Threading;

namespace TcpServer
{
    class Program
    {
        private static AutoResetEvent are = new AutoResetEvent(false);
        private static ConcurrentQueue<byte[]> sendMessages = new ConcurrentQueue<byte[]>();
        private static long nextHeartbeatSendTime;
        private static long nextSensorSendTime;
        private static TcpClient sender;
        private static bool running = true;
        private static MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();
        private static int sendSequence = 0;
        private static int sensorSequence = 0;
        private static long startTime;

        public static void Main(string[] args)
        {
            startTime = DateTime.UtcNow.Ticks;

            IPEndPoint ep = new IPEndPoint(IPAddress.Any, 5760);
            TcpListener listener = new TcpListener(ep);
            listener.Start();

            Console.WriteLine(@"Address: " + ep.Address + "\t Port: " + ep.Port);
            sender = listener.AcceptTcpClient();
            Console.WriteLine("Connected");

            Thread receiveThread = new Thread(new ThreadStart(ReceiveMain));
            receiveThread.Start();
            Thread sendThread = new Thread(new ThreadStart(SendMain));
            sendThread.Start();

            while (running)
            {
                if (Console.KeyAvailable)
                {
                    ConsoleKeyInfo cki = Console.ReadKey(false);
                    if (cki.KeyChar == 'q')
                    {
                        running = false;
                    }
                }
                Thread.Sleep(50);
            }

            sendThread.Join();
            receiveThread.Join();
            Console.WriteLine("Bye!");
        }

        private static void SendMessage(MAVLink.MAVLINK_MSG_ID messageType, object message)
        {
            byte[] sendBytes = parser.GenerateMAVLinkPacket10(messageType, message, 1, (byte)MAVLink.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1, sendSequence);
            sendSequence++;
            sendMessages.Enqueue(sendBytes);
        }

        private static void ProcessMessage(MAVLink.MAVLinkMessage message)
        {
            Console.WriteLine($"Receiving {message.msgtypename} {message.payloadlength}");
        }

        private static void CheckSendHeartbeat()
        {
            long currentTime = DateTime.UtcNow.Ticks;
            if (currentTime > nextHeartbeatSendTime)
            {
                nextHeartbeatSendTime = currentTime + TimeSpan.TicksPerSecond;
                MAVLink.mavlink_heartbeat_t message = new MAVLink.mavlink_heartbeat_t(0, (byte)MAVLink.MAV_TYPE.FIXED_WING, (byte)MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA, (byte)MAVLink.MAV_MODE.AUTO_ARMED, (byte)MAVLink.MAV_STATE.ACTIVE, (byte)MAVLink.MAVLINK_VERSION);
                SendMessage(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, message);

                uint sensors = (uint)(MAVLink.MAV_SYS_STATUS_SENSOR._3D_GYRO | MAVLink.MAV_SYS_STATUS_SENSOR._3D_ACCEL | MAVLink.MAV_SYS_STATUS_SENSOR._3D_MAG | MAVLink.MAV_SYS_STATUS_SENSOR.ABSOLUTE_PRESSURE | MAVLink.MAV_SYS_STATUS_SENSOR.BATTERY | MAVLink.MAV_SYS_STATUS_SENSOR.GPS);
                MAVLink.mavlink_sys_status_t sysStatus = new MAVLink.mavlink_sys_status_t(sensors, sensors, sensors, 10, 11000, 300, 0, 0, 0, 0, 0, 0, 34);
                SendMessage(MAVLink.MAVLINK_MSG_ID.SYS_STATUS, message);

                ulong unixTime = (ulong)(DateTime.UtcNow - DateTime.UnixEpoch).TotalSeconds;
                uint uptime = (uint)((DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond);
                MAVLink.mavlink_system_time_t sysTime = new MAVLink.mavlink_system_time_t(unixTime, uptime);
                SendMessage(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME, sysTime);
            }
        }

        private static void CheckSensor()
        {
            long currentTime = DateTime.UtcNow.Ticks;
            if (currentTime > nextSensorSendTime)
            {
                uint uptime = (uint)((DateTime.UtcNow.Ticks - startTime) / TimeSpan.TicksPerMillisecond);

                nextSensorSendTime = currentTime + 500 * TimeSpan.TicksPerMillisecond;
                ushort[] voltages = new ushort[10];
                voltages[0] = 3700;
                voltages[1] = 3750;
                voltages[2] = 3650;
                ushort[] voltages2 = new ushort[4];
                MAVLink.mavlink_battery_status_t batMsg = new MAVLink.mavlink_battery_status_t(sendSequence, -1, 6000, voltages, 3, 0, (byte)MAVLink.MAV_BATTERY_FUNCTION.ALL, (byte)MAVLink.MAV_BATTERY_TYPE.LIPO, 66, 1000 - sensorSequence, (byte)MAVLink.MAV_BATTERY_CHARGE_STATE.OK, voltages2, (byte)MAVLink.MAV_BATTERY_MODE.UNKNOWN, 0);
                SendMessage(MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS, batMsg);

                MAVLink.mavlink_global_position_int_t gpsMsg = new MAVLink.mavlink_global_position_int_t(uptime, 1 + sensorSequence / 1000, 1 + sensorSequence / 1000, 9001, 9001, 51, 52, 0, (ushort)(sensorSequence % 360));
                SendMessage(MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, gpsMsg);

                byte[] prns = new byte[20];
                byte[] used = new byte[20];
                byte[] ele = new byte[20];
                byte[] azith = new byte[20];
                byte[] snr = new byte[20];
                for (int i = 0; i < 10; i++)
                {
                    snr[i] = 10;
                    if (i <= 5)
                    {
                        used[i] = 1;
                        snr[i] = 40;
                    }
                    prns[i] = (byte)i;
                    ele[i] = (byte)(30 + i * 5);
                    azith[i] = (byte)(10 + i * 10);

                }
                MAVLink.mavlink_gps_status_t gpsStatusMsg = new MAVLink.mavlink_gps_status_t((byte)5, prns, used, ele, azith, snr);
                SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_STATUS, gpsStatusMsg);

                MAVLink.mavlink_gps_raw_int_t gpsRaw = new MAVLink.mavlink_gps_raw_int_t(uptime, 100000, 1000000, 9000, 300,300,900, 9000, (byte)MAVLink.GPS_FIX_TYPE._3D_FIX, 5, 9000, 0, 0, 0, 0, 0);
                SendMessage(MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT, gpsRaw);

                MAVLink.mavlink_attitude_t attitudeMsg = new MAVLink.mavlink_attitude_t(uptime, -1 + (sensorSequence / 20f), -1 + (sensorSequence / 20f), -1 + (sensorSequence / 100f), 0, 0, 0);
                SendMessage(MAVLink.MAVLINK_MSG_ID.ATTITUDE, attitudeMsg);
                

                //MAVLink.mavlink_radio_status_t radioMsg = new MAVLink.mavlink_radio_status_t(0, 0, 254, 254, 100, 255, 255);
                //SendMessage(MAVLink.MAVLINK_MSG_ID.RADIO_STATUS, radioMsg);

                MAVLink.mavlink_rc_channels_t rcChannels = new MAVLink.mavlink_rc_channels_t(uptime, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 4, 200);
                SendMessage(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS, rcChannels);

                sensorSequence++;
            }
        }

        private static void ReceiveMain()
        {
            byte[] buffer = new byte[263];
            bool readingHeader = true;
            //Packet framing is 8 bytes
            int bytesLeft = 8;
            int readPos = 0;
            NetworkStream ns = sender.GetStream();
            while (running)
            {
                try
                {
                    int bytesRead = ns.Read(buffer, readPos, bytesLeft);
                    readPos += bytesRead;
                    bytesLeft -= bytesRead;
                    if (bytesRead == 0)
                    {
                        //Disconnect
                        running = false;
                        return;
                    }
                    if (bytesLeft == 0)
                    {
                        if (readingHeader)
                        {
                            bytesLeft = buffer[1];
                            if (bytesLeft == 0)
                            {
                                //Process 0 byte messages
                                MAVLink.MAVLinkMessage mlm = new MAVLink.MAVLinkMessage(buffer);
                                ProcessMessage(mlm);
                                readingHeader = true;
                                readPos = 0;
                                bytesLeft = 8;
                            }
                            else
                            {
                                readingHeader = false;
                            }
                        }
                        else
                        {
                            //Process messages with a payload
                            MAVLink.MAVLinkMessage mlm = new MAVLink.MAVLinkMessage(buffer);
                            ProcessMessage(mlm);
                            readingHeader = true;
                            readPos = 0;
                            bytesLeft = 8;
                        }
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine("Receive error: " + e.Message);
                    running = false;
                }
            }
        }

        private static void SendMain()
        {
            NetworkStream ns = sender.GetStream();
            while (running)
            {
                are.WaitOne(50);
                if (sendMessages.TryDequeue(out byte[] message))
                {
                    MAVLink.MAVLinkMessage sendMessageTest = new MAVLink.MAVLinkMessage(message);
                    Console.WriteLine($"Sending {sendMessageTest.msgtypename} {sendMessageTest.payloadlength}");
                    try
                    {
                        ns.Write(message, 0, message.Length);
                    }
                    catch
                    {
                        running = false;
                        return;
                    }
                }
                CheckSendHeartbeat();
                CheckSensor();
            }
        }
    }
}

// |===============================|
// ||          TODO LIST          ||
// |===============================|
// ||     MAKE HEARTBEAT (1hz)    ||
// ||  INTERPRET INCOMMING DATA   ||
// ||  GET RID OF THE WHILE LOOP  ||
// ||  SEND ACK TO GCS AS RETURN  ||
// ||                             ||
// ||                             ||
// ||                             ||
// ||                             ||
// ||                             ||
// |===============================|

// |===============================|
// ||   TODO LIST LONG(ER) TERM   ||
// |===============================|
// ||  GET PLANE ROLL AND PITCH   ||
// ||    MAKE A SOFTQARE IMU      ||
// ||     GET PLANE ALTITUDE      ||
// ||     GET PLANE THROTTLE      ||
// ||  GET BATTERY/FUEL REMANING  ||
// ||                             ||
// ||                             ||
// ||                             ||
// |===============================|