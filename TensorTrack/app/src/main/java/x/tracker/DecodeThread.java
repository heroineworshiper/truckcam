package x.tracker;


import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

class DecodeThread implements Runnable
{
    FirstFragment fragment;

    DecodeThread(FirstFragment fragment)
    {
        this.fragment = fragment;
    }

    @Override
    public void run() {
        byte[] buffer = new byte[0x10000];


        try {
            fragment.ffmpeg_stdout = new FileInputStream(new File(fragment.stdoutPath));

        } catch (IOException e) {
            e.printStackTrace();
        }

        int inputOffset = 0;
        int outputOffset = 0;
        byte[] dst = fragment.frameBuffer[fragment.currentFrameBuffer].array();
        while (true) {
            int bytes_read = 0;
            try {
                bytes_read = fragment.ffmpeg_stdout.read(buffer, inputOffset, buffer.length - inputOffset);


                int pixels = (inputOffset + bytes_read) / 3;
                if (outputOffset / 4 + pixels > fragment.W * fragment.H) {
                    pixels = fragment.W * fragment.H - outputOffset / 4;
                }

//                        Log.i("DecodeThread", "bytes_read=" + bytes_read + " pixels=" + pixels + " outputOffset=" + outputOffset);
                // transfer pixels to bitmap
                for (int i = 0; i < pixels; i++) {
                    dst[outputOffset++] = (byte) buffer[i * 3];
                    dst[outputOffset++] = (byte) buffer[i * 3 + 1];
                    dst[outputOffset++] = (byte) buffer[i * 3 + 2];
                    dst[outputOffset++] = (byte) 0xff;
                }

                // shift input buffer
                inputOffset = (inputOffset + bytes_read) - pixels * 3;
                for (int i = 0; i < inputOffset; i++) {
                    buffer[i] = buffer[pixels * 3 + i];
                }

                if (outputOffset >= fragment.W * fragment.H * 4) {
                    outputOffset = 0;
                    fragment.currentFrameBuffer++;
                    if (fragment.currentFrameBuffer > 1) {
                        fragment.currentFrameBuffer = 0;
                    }
                    dst = fragment.frameBuffer[fragment.currentFrameBuffer].array();
                    fragment.drawVideo();
                }
            } catch(IOException e){
                e.printStackTrace();
            }
        }
    }
};

