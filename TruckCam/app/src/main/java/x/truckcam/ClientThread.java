package x.truckcam;

import android.graphics.Bitmap;
import android.graphics.ImageDecoder;
import android.graphics.drawable.Drawable;
import android.util.Log;

import com.arthenica.ffmpegkit.ExecuteCallback;
import com.arthenica.ffmpegkit.FFmpegKit;
import com.arthenica.ffmpegkit.Session;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.Formatter;
import java.util.StringTokenizer;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

class ClientThread implements Runnable {
    private FirstFragment fragment;

    static final String SERVER = "10.0.3.1";
    //static final String SERVER = "10.0.0.17";
    static final int PORT0 = 1234;
    static final int PORT1 = 1238;

    static Socket socket;
    byte[] header = new byte[8];
    byte[] packet = new byte[1024 * 1024];

    final int GET_START_CODE0 = 0;
    final int GET_START_CODE1 = 1;
    final int GET_HEADER = 2;
    final int GET_DATA = 3;
    int packetState = GET_START_CODE0;
    int counter = 0;
    int dataSize = 0;

    final int START_CODE0 = 0xff;
    final int START_CODE1 = 0xe7;

    // packet type
    final int VIJEO = 0x00;
    final int STATUS = 0x01;
    ExecutorService pool = Executors.newFixedThreadPool(1);

    ClientThread(FirstFragment fragment)
    {
        this.fragment = fragment;
    }


    public static int write_int32(byte[] data, int offset, int value) {
        data[offset++] = (byte)(value & 0xff);
        data[offset++] = (byte)((value >> 8) & 0xff);
        data[offset++] = (byte)((value >> 16) & 0xff);
        data[offset++] = (byte)((value >> 24) & 0xff);
        return offset;
    }

    static public int read_int32(byte[] data, int offset)
    {
        return (data[offset] & 0xff) |
                ((data[offset + 1] & 0xff) << 8) |
                ((data[offset + 2] & 0xff) << 16) |
                ((data[offset + 3]) << 24);
    }


    static public void printBuffer(String string, byte[] buffer, int offset, int bytes)
    {
        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb);
        for(int i = 0; i < bytes; i++)
        {
            formatter.format("%02x ", buffer[i + offset]);
        }
        Log.i(string, sb.toString());
    }

    private void handleStatus() {
        if(fragment.currentOperation != fragment.prevOperation ||
            fragment.prevLandscape != fragment.landscape)
        {
            fragment.changeOperation();
        }
        else
        {
            fragment.updateValues();
        }
    }

    void sendCommand(int id) {
        pool.execute(new Runnable() {
            @Override
            public void run() {

                synchronized (this) {
                    if (socket != null) {
                        byte[] buffer = new byte[1];
                        buffer[0] = (byte) id;
                        try {
                            socket.getOutputStream().write(buffer);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }
            }
        });
    }

    public void sendSettings() {
        pool.execute(new Runnable() {

            @Override
            public void run() {
                synchronized(this)
                {
                    File file = new File("/sdcard/truckcam/settings.txt");
                    BufferedReader reader = null;
                    byte[] buffer = new byte[6];
                    buffer[0] = 's';
                    buffer[1] = (byte)0xff;
                    buffer[2] = (byte)0xff;
                    buffer[3] = (byte)0xff;
                    buffer[4] = (byte)0xff;
                    buffer[5] = (byte)0xff;
                    try {
                        reader = new BufferedReader(new FileReader(file));
                        String line = null;
                        while ((line = reader.readLine()) != null)
                        {
//				Log.v("Settings", "loadInternal " + line);

                            StringTokenizer st = new StringTokenizer(line);
                            Vector<String> strings = new Vector<String>();
                            while(st.hasMoreTokens())
                            {
                                strings.add(st.nextToken());
                            }

                            // comment or whitespace
                            if(strings.size() < 2 || strings.get(0).charAt(0) == '#') continue;

                            String key = strings.get(0);
                            String value = strings.get(1);
                            if(key.equalsIgnoreCase("DEADBAND"))
                            {
                                buffer[1] = (byte)Integer.parseInt(value);
                            }
                            else
                            if(key.equalsIgnoreCase("SPEED"))
                            {
                                buffer[2] = (byte)Integer.parseInt(value);
                            }
                            else
                            if(key.equalsIgnoreCase("XY_RADIUS"))
                            {
                                buffer[3] = (byte)Integer.parseInt(value);
                            }
                            else
                            if(key.equalsIgnoreCase("SIZE_RADIUS"))
                            {
                                buffer[4] = (byte)Integer.parseInt(value);
                            }
                            else
                            if(key.equalsIgnoreCase("COLOR_RADIUS"))
                            {
                                buffer[5] = (byte)Integer.parseInt(value);
                            }
                        }
                        reader.close();
                        socket.getOutputStream().write(buffer);

                    } catch (Exception e) {
                        Log.i("ClientThread", "sendSettings 1 " + e.toString());
                        return;
                    }



                }
            }
        });
    }


    @Override
    public void run() {
        fragment.waitForSurface();


        // connect to the server
        int currentPort = PORT0;
        while (true) {
            Log.i("ClientThread", "server=" + SERVER + ":" + currentPort);
            fragment.drawStatus("Trying " + SERVER + ":" + currentPort);
            try {
                InetAddress serverAddr = InetAddress.getByName(SERVER);
                synchronized(this) {
                    socket = new Socket(serverAddr, currentPort);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }

            if (socket == null) {
                Log.i("ClientThread", "Couldn't access server " + SERVER + ":" + currentPort);
                currentPort++;
                if (currentPort >= PORT1) {
                    currentPort = PORT0;
                }
                try {
                    Thread.sleep(1000L);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {


                fragment.drawStatus("Reading stream");

                if(FirstFragment.USE_FFMPEG)
                {
                    //String command = "-probesize 32 -vcodec h263 -y -i " + stdinPath + " -vcodec rawvideo -f rawvideo -flush_packets 1 -pix_fmt rgb24 " + stdoutPath;
                    //String command = "-probesize 32 -vcodec hvec -y -i " + stdinPath + " -vcodec rawvideo -f rawvideo -flush_packets 1 -pix_fmt rgb24 " + stdoutPath;
                    String command = "-probesize 32 -vcodec mjpeg -y -i " + fragment.stdinPath + " -vcodec rawvideo -f rawvideo -flush_packets 1 -pix_fmt rgb24 " + fragment.stdoutPath;
                    Log.i("ClientThread", "Running " + command);


                    FFmpegKit.executeAsync(command,
                            new ExecuteCallback() {
                                @Override
                                public void apply(Session session) {
                                }
                            });


                    try {
                        fragment.ffmpeg_stdin = new FileOutputStream(fragment.stdinPath);

                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }


                // read stream from server
                byte[] buffer = new byte[1024];
                int total = 0;
                while (true) {
                    int bytes_read = 0;
                    try {
                        bytes_read = socket.getInputStream().read(buffer);

                        //Log.i("ClientThread", "bytes_read=" + bytes_read);
                        if (bytes_read < 0)
                        {
                            break;
                        }



                        for(int i = 0; i < bytes_read; i++)
                        {
                            int c = (int)buffer[i];
                            switch(packetState)
                            {
                                case GET_START_CODE0:
                                    if(c == (byte)START_CODE0)
                                    {
                                        //Log.i("ClientThread", "START_CODE0");
                                        packetState = GET_START_CODE1;
                                    }
                                    break;
                                case GET_START_CODE1:

                                    if(c == (byte)START_CODE1)
                                    {
                                        //Log.i("ClientThread", "GET_START_CODE1");

                                        packetState = GET_HEADER;
                                        counter = 2;
                                    }
                                    else
                                    if(c == (byte)START_CODE0)
                                    {
                                        packetState = GET_START_CODE1;
                                    }
                                    else
                                    {
                                        packetState = GET_START_CODE0;
                                    }
                                    break;
                                case GET_HEADER:
                                    header[counter++] = (byte)c;
                                    if(counter >= 8)
                                    {
                                        //Log.i("ClientThread", "GET_HEADER ");
                                        dataSize = read_int32(header, 4);
                                        packetState = GET_DATA;
                                        counter = 0;
                                    }
                                    break;
                                case GET_DATA:
                                    if(counter >= packet.length)
                                    {
                                        counter--;
                                    }
                                    packet[counter++] = (byte)c;
                                    if(counter >= dataSize)
                                    {
                                        int type = (int)header[2];

                                        //Log.i("ClientThread", "GET_DATA type=" + type);
                                        if(type == VIJEO)
                                        {
                                            total += dataSize;
                                            //Log.i("ClientThread", "VIJEO dataSize=" + dataSize);
                                            if(FirstFragment.USE_FFMPEG) {
                                                fragment.ffmpeg_stdin.write(packet, 0, dataSize);
                                            }
                                            else
                                            {
                                                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.P) {
                                                    ImageDecoder.Source imageSource =
                                                        ImageDecoder.createSource(ByteBuffer.wrap(packet, 0, dataSize));
                                                    // generates a hardware bitmap
                                                    Bitmap bitmap = ImageDecoder.decodeBitmap(imageSource);
//                                                    Log.i("x", "VIJEO w=" + bitmap.getWidth() +
//                                                            " h=" + bitmap.getHeight() +
//                                                            " " + bitmap.getColorSpace());
                                                    fragment.drawVideo(bitmap);
                                                }
                                            }
                                        }
                                        else if(type == STATUS)
                                        {
                                            fragment.prevOperation = fragment.currentOperation;
                                            fragment.prevLandscape = fragment.landscape;

                                            fragment.currentOperation = packet[0];
                                            fragment.facePosition = packet[1];
                                            fragment.errors = packet[2] & 0xff;

                                            Log.i("ClientThread", "GET_DATA" +
                                                    " currentOperation=" + fragment.currentOperation +
                                                    " errors=" + fragment.errors);

                                            handleStatus();
                                        }


                                        packetState = GET_START_CODE0;
                                    }
                                    break;
                            }

                        }


                        //Log.i("ClientThread", " total=" + total);

                    } catch (IOException e) {
                        e.printStackTrace();
                        break;
                    }

                }

                Log.i("ClientThread", "connection finished");
                try {
                    synchronized(this) {
                        socket.close();
                        socket = null;
                    }
                    Thread.sleep(1000L);
                } catch (Exception e) {
                    e.printStackTrace();
                }

            }
        }
    }

}
