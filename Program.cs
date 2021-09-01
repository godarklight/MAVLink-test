using System;
using System.Net.Sockets;
using System.Net;
using System.Threading.Tasks;
using System.Diagnostics;

namespace TcpServer
{
    [DebuggerDisplay("{" + nameof(GetDebuggerDisplay) + "(),nq}")]
    class Program
    {
        static void Main(string[] args)
        {
            MAVLink mav = new MAVLink();
            byte[] buffer = new byte[263];
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, 5760);
            TcpListener listener = new TcpListener(ep);
            listener.Start();

            Console.WriteLine(@"Address: " + ep.Address + "\t Port: " + ep.Port);
            TcpClient sender = listener.AcceptTcpClient();
            Console.WriteLine("Connected");

            // Run the loop continously; this is the server.
            bool readingHeader = true;
            int bytesLeft = 8;
            int readPos = 0;
            while (true)
            {
                int bytesRead = sender.GetStream().Read(buffer, readPos, bytesLeft);
                readPos += bytesRead;
                bytesLeft -= bytesRead;
                if (bytesRead == 0)
                {
                    //Disconnect
                    return;
                }
                if (bytesLeft == 0)
                {
                    if (readingHeader)
                    {
                        readingHeader = false;
                        bytesLeft = buffer[1];
                    }
                    else
                    {
                        //Process buffer here
                        MAVLink.MAVLinkMessage mlm = new MAVLink.MAVLinkMessage(buffer);
                        string hexString = BitConverter.ToString(buffer, 0, readPos);
                        Console.WriteLine(mlm.msgtypename + ": " + hexString.Replace('-', ' '));
                        Console.WriteLine("===");

                        readingHeader = true;
                        readPos = 0;
                        bytesLeft = 8;
                    }
                }

                // I have no clue on how to make it into a MAVLink packet
                //var packet = MavlinkUtil.ByteArrayToStructure


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

            }
        }

        private string GetDebuggerDisplay()
        {
            return ToString();
        }
    }
}