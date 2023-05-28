using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;

public class MyListener : MonoBehaviour
{
    Thread thread;
    public int connectionPort = 25001;
    TcpListener server;
    TcpClient client;
    bool running;
    string dataReceived;


    void Start()
    {
        // Receive on a separate thread so Unity doesn't freeze waiting for data
        ThreadStart ts = new ThreadStart(GetData);
        thread = new Thread(ts);
        thread.Start();
    }

    void GetData()
    {
        // Create the server
        server = new TcpListener(IPAddress.Any, connectionPort);
        server.Start();

        // Create a client to get the data stream
        client = server.AcceptTcpClient();

        // Start listening
        running = true;
        while (running)
        {
            Connection();
        }
        server.Stop();
    }

    void Connection()
    {
        // Read data from the network stream
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);

        // Decode the bytes into a string
        dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead);
        
        // Make sure we're not getting an empty string
        //dataReceived.Trim();
        if (dataReceived != null && dataReceived != "")
        {
            // Convert the received string of data to the format we are using
            position = ParseData(dataReceived);
            nwStream.Write(buffer, 0, bytesRead);
        }
    }

    // Use-case specific function, need to re-write this to interpret whatever data is being sent
    public static Vector3 ParseData(string dataString)
    {
        Debug.Log(dataString);
        // Remove the parentheses
        if (dataString.StartsWith("(") && dataString.EndsWith(")"))
        {
            dataString = dataString.Substring(1, dataString.Length - 2);
        }

        // Split the elements into an array
        string[] stringArray = dataString.Split(',');

        // Store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(stringArray[0]),
            float.Parse(stringArray[1]),
            float.Parse(stringArray[2]));

        return result;
    }
    

    // Position is the data being received in this example
    Vector3 position = Vector3.zero;

    void Update()
    {
        string dataString =0;
        dataString = dataReceived;
        Debug.Log(dataString);

        // Remove the parentheses
        if (dataString.StartsWith("(") && dataString.EndsWith(")"))
        {
            dataString = dataString.Substring(1, dataString.Length - 2);
        }

        // Split the elements into an array
        string[] stringArray = dataString.Split(',');
        // Debug.Log(stringArray[0]);
    //     Debug.Log(stringArray[1]);
    //     Debug.Log(stringArray[2]);
    //     Debug.Log(stringArray[3]);
    //     Debug.Log(stringArray[4]);
    //     Debug.Log(stringArray[5]);
        // // Store as a Vector3
        // Vector3 gyro = new Vector3(
        //     float.Parse(stringArray[0]),
        //     float.Parse(stringArray[1]),
        //     float.Parse(stringArray[2]));

        // // Store as a Vector3
        //  Vector3 accel = new Vector3(
        //     float.Parse(stringArray[3]),
        //     float.Parse(stringArray[4]),
        //     float.Parse(stringArray[5]));

        // // Calculate the rotation
        // Quaternion gyroRotation = Quaternion.Euler(gyro * Time.deltaTime);
        // Quaternion accelRotation = Quaternion.FromToRotation(transform.up, accel) * transform.rotation;
        // Quaternion totalRotation = Quaternion.Lerp(gyroRotation, accelRotation, 0.5f);

        // // Apply the rotation to the cube
        // transform.rotation = totalRotation;
    }
}