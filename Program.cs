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
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, 1234);
            TcpListener listener = new TcpListener(ep);
            listener.Start();

            Console.WriteLine(@"Address: " + ep.Address + "\t Port: " + ep.Port);

            // Run the loop continously; this is the server.  
            while (true)
            {
                const int bytesize = 263;
                byte[] buffer = new byte[bytesize];

                var sender = listener.AcceptTcpClient();
                sender.GetStream().Read(buffer, 0, bytesize);

                // Trimming the bufferr before dumping it to the console
                byte[] bufferTrimmed = MavlinkUtil.trim_payload(ref buffer);
                string hexString = BitConverter.ToString(bufferTrimmed);
                Console.WriteLine(hexString.Replace('-', ' '));

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