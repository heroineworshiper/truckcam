package x.truckcam;

import android.app.ActionBar;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.VideoView;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.fragment.NavHostFragment;
import x.truckcam.databinding.FragmentFirstBinding;

import static android.content.Context.MODE_APPEND;
import static android.content.Context.TELECOM_SERVICE;

public class FirstFragment extends Fragment implements View.OnTouchListener {

    private FragmentFirstBinding binding;
    SurfaceView video;
//    ByteBuffer[] frameBuffer = new ByteBuffer[2];
//    int currentFrameBuffer = 0;
    Bitmap videoBitmap;
    Canvas videoCanvas;
    ClientThread client;


    // size of the cropped preview video
    static final int W = 640;
    static final int H = 360;
    static OutputStream ffmpeg_stdin;
    static InputStream ffmpeg_stdout;
    static String stdinPath;
    static String stdoutPath;



    final int OFF = -1;
    final int IDLE  = 0;
    final int TRACKING = 1;
    int currentOperation = OFF;
    int prevOperation = OFF;
    final int FACE_LEFT = 0;
    final int FACE_CENTER = 1;
    final int FACE_RIGHT = 2;
    int facePosition = FACE_CENTER;

    static boolean landscape = true;
    static boolean prevLandscape = false;

// error codes
    final int VIDEO_DEVICE_ERROR = 1;
    final int VIDEO_BUFFER_ERROR = 2;
    final int SERVO_ERROR = 4;

    int errors;

    float accelX = 0;
    float accelY = 0;
    final float ACCEL_BANDWIDTH = (float)0.2;

    Vector<Button> buttons = new Vector();
    Vector<Text> texts = new Vector();
    Button activateButton;
//    Button leftButton;
//    Button centerButton;
//    Button rightButton;
    Text videoDeviceError;
    Text videoBufferError;
    Text servoError;
    static final int MARGIN = 40;
    static final int ARROW_MARGIN = 20;
    static final int TEXT_SIZE = 40;

    @Override
    public View onCreateView(
            LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState
    ) {
        // landscape mode with no status bar
//        getActivity().setRequestedOrientation(
//                ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
//        View decorView = getActivity().getWindow().getDecorView();
//// Hide the status bar in landscape mode.
//        int uiOptions = View.SYSTEM_UI_FLAG_FULLSCREEN;
//        decorView.setSystemUiVisibility(uiOptions);

//        ActionBar actionBar = getActivity().getActionBar();
//        actionBar.hide();

        binding = FragmentFirstBinding.inflate(inflater, container, false);
        video = binding.surfaceView2;





      return binding.getRoot();

    }

    public void onViewCreated(@NonNull View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        binding.buttonFirst.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                NavHostFragment.findNavController(FirstFragment.this)
                        .navigate(R.id.action_FirstFragment_to_SecondFragment);
            }
        });
        // attach the listener
        video.setOnTouchListener(this);



        videoBitmap = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);
        videoCanvas = new Canvas();
        videoCanvas.setBitmap(videoBitmap);

//        for(int i = 0; i < 2; i++) {
//            frameBuffer[i] = ByteBuffer.allocateDirect(videoBitmap.getByteCount());
//        }


//        if(USE_FFMPEG) {
//            new Thread(new DecodeThread(this)).start();
//        }
        new Thread(client = new ClientThread(this)).start();

    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    // wait for the surface view to be created
    boolean gotIt = false;
    public void waitForSurface()
    {
        while(!gotIt)
        {
            try {
                Thread.sleep(100L);
            } catch (Exception e) {
                e.printStackTrace();
            }

            getActivity().runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Canvas canvas = video.getHolder().lockCanvas();
                        if(canvas != null) {



                            video.getHolder().unlockCanvasAndPost(canvas);
                            gotIt = true;
                        }
                    }
                }
            );
        }
    }


    public void drawStatus(String text)
    {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();
                if(canvas != null) {
                    Paint p = new Paint();
                    p.setColor(Color.GREEN);
                    p.setStyle(Paint.Style.FILL);
                    p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
                    p.setTextSize(TEXT_SIZE);
                    Rect text_size = new Rect();
                    p.getTextBounds(text,
                               0,
                               text.length(),
                               text_size);
                    if(landscape) {
                        // make a temporary canvas for rotating the text
                        Bitmap temp2 = Bitmap.createBitmap(text_size.width(), text_size.height(), Bitmap.Config.ARGB_8888);
                        Canvas temp = new Canvas(temp2);
                        temp.drawText(text, 0, temp.getHeight() - text_size.bottom, p);

                        // rotate & draw it
                        Matrix matrix = new Matrix();
                        matrix.reset();
                        matrix.postTranslate(-temp.getWidth() / 2, -temp.getHeight() / 2); // Centers image
                        matrix.postRotate(90);
                        matrix.postTranslate(canvas.getWidth() / 2, canvas.getHeight() / 2);

                        p.setColor(Color.BLACK);
                        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);
                        canvas.drawBitmap(temp2,
                                matrix,
                                p);
                    }
                    else
                    {
                        p.setColor(Color.BLACK);
                        canvas.drawRect(new Rect(0,
                                canvas.getHeight() / 2 - text_size.height() / 2,
                                canvas.getWidth(),
                                canvas.getHeight() / 2 + text_size.height() / 2), p);
                        p.setColor(Color.GREEN);
                        canvas.drawText(text,
                                canvas.getWidth() / 2 - text_size.width() / 2,
                                canvas.getHeight() / 2 + text_size.height() / 2 - text_size.bottom,
                                p);

                    }

//Log.i("FirstFragment", "drawStatus " + text);
                    video.getHolder().unlockCanvasAndPost(canvas);
                }
                else
                {
                    Log.i("drawStatus", "No canvas");
                }
            }
        });
    }

    public void drawVideo(Bitmap bitmap, int preview_x)
    {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();

                if(canvas != null) {
                    // overlay cropped section
                    Paint p = new Paint();
                    // must convert to a software bitmap for draw()
                    Bitmap softBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, false);
// ignore preview_x to display the entire viewfinder
                    //videoCanvas.drawBitmap(softBitmap, preview_x, 0, p);
                    videoCanvas.drawBitmap(softBitmap, 0, 0, p);
                    drawGUI(canvas);
                    video.getHolder().unlockCanvasAndPost(canvas);
                }
            }
        });
    }

//    public void drawVideo() {
//        getActivity().runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                Canvas canvas = video.getHolder().lockCanvas();
//
//                if(canvas != null) {
////                    Paint p = new Paint();
////                    p.setColor(Color.BLACK);
////                    p.setStyle(Paint.Style.FILL);
////
////                    canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);
//
//                    int current = currentFrameBuffer - 1;
//                    if (current < 0)
//                    {
//                        current = 1;
//                    }
//                    videoBitmap.copyPixelsFromBuffer(frameBuffer[current]);
//                    frameBuffer[current].rewind();
//                    drawGUI(canvas);
//
//                    video.getHolder().unlockCanvasAndPost(canvas);
//                }
//            }
//        });
//    }

    boolean updateErrors()
    {
        boolean needRedraw = false;
        needRedraw |= videoDeviceError.setHidden((errors & VIDEO_DEVICE_ERROR) == 0);
        needRedraw |= videoBufferError.setHidden((errors & VIDEO_BUFFER_ERROR) == 0);
        needRedraw |= servoError.setHidden((errors & SERVO_ERROR) == 0);
        return needRedraw;
    }

    public void updateValues() {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                boolean needRedraw = false;
//                if(leftButton != null)
//                {
//                    needRedraw |= leftButton.updateText(getLeftText());
//                }
//                if(centerButton != null)
//                {
//                    needRedraw |= centerButton.updateText(getCenterText());
//                }
//                if(rightButton != null)
//                {
//                    needRedraw |= rightButton.updateText(getRightText());
//                }

                needRedraw |= updateErrors();

                if(needRedraw) {
                    Canvas canvas = video.getHolder().lockCanvas();
                    if (canvas != null) {
                        drawGUI(canvas);
                        video.getHolder().unlockCanvasAndPost(canvas);
                    }
                }
            }
        });
    }

    public void changeOperationSync()
    {
        buttons.clear();
        texts.clear();

        activateButton = null;
//        leftButton = null;
//        centerButton = null;
//        rightButton = null;



        Canvas canvas = video.getHolder().lockCanvas();

        if (canvas != null) {
            int x, y;
            Rect size = Text.calculateSize("X");
            if(landscape) {
                x = canvas.getWidth() / 4;
                y = canvas.getHeight() / 2;
            }
            else
            {
                x = MARGIN;
                y = size.height() / 2 + MARGIN;
            }


            videoDeviceError = new Text(x, y, "VIDEO DEVICE NOT FOUND");
            videoDeviceError.color = Color.RED;
            if(landscape) {
                x += canvas.getWidth() / 4;
            }
            else
            {
                y += canvas.getHeight() / 8;
            }
            videoBufferError = new Text(x, y, "VIDEO CAPTURE FAILED");
            videoBufferError.color = Color.RED;
            if(landscape) {
                x += canvas.getWidth() / 4;
            }
            else
            {
                y += canvas.getHeight() / 8;
            }
            servoError = new Text(x, y, "SERVO DEVICE NOT FOUND");
            servoError.color = Color.RED;

            texts.add(videoDeviceError);
            texts.add(videoBufferError);
//            texts.add(servoError);
            updateErrors();

            switch(currentOperation)
            {
                case IDLE:
                case TRACKING: {
                    String text;
                    if(currentOperation == IDLE) {
                        text = "ACTIVATE SERVO";
                    }
                    else{
                        text = "ABORT";
                    }
                    size = Button.calculateSize(text, null);
                    if(landscape) {
                        x = canvas.getWidth() - size.width() * 2 - MARGIN;
                        y = MARGIN + size.height() / 2;
                    }
                    else
                    {
                        x = canvas.getWidth() / 2;
                        y = canvas.getHeight() * 3 / 4;
                    }


                    activateButton = new Button(x, y, text);
                    activateButton.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "ACTIVATE SERVO");
                            if(currentOperation == IDLE) {
                                client.sendCommand(' ');
                            }
                            else{
                                client.sendCommand('q');
                            }
                        }
                    };
                    buttons.add(activateButton);

                    y = MARGIN;
//                    text = "FACE POSITION:";
//                    size = Text.calculateSize(text);
//                    x -= activateButton.getW() + MARGIN;
//                    Text t = new Text(x, y, text);
//                    texts.add(t);
//
//                    text = getLeftText();
//                    size = Button.calculateSize("O", null);
//                    x -= t.getW() + MARGIN + size.width() / 2;
//                    y = MARGIN + size.height() / 2;
//                    leftButton = new Button(x, y, text);
//                    leftButton.listener = new Button.ButtonListener() {
//                        @Override
//                        public void onClick() {
//                            Log.i("FirstFragment", "LEFT");
//                            client.sendCommand('l');
//                        }
//                    };
//                    buttons.add(leftButton);
//
//                    y += MARGIN + size.height() * 2;
//                    centerButton = new Button(x, y, getCenterText());
//                    centerButton.listener = new Button.ButtonListener() {
//                        @Override
//                        public void onClick() {
//                            Log.i("FirstFragment", "CENTER");
//                            client.sendCommand('c');
//                        }
//                    };
//
//                    buttons.add(centerButton);
//                    y += MARGIN + size.height() * 2;
//                    rightButton = new Button(x, y, getRightText());
//                    rightButton.listener = new Button.ButtonListener() {
//                        @Override
//                        public void onClick() {
//                            Log.i("FirstFragment", "RIGHT");
//                            client.sendCommand('r');
//                        }
//                    };
//                    buttons.add(rightButton);

//                    x -= MARGIN + size.width() * 2;
                    x = MARGIN + size.width() * 2;
                    text = "UPDATE SETTINGS";
                    size = Button.calculateSize(text, null);
                    Button b = new Button(x, MARGIN + size.height() / 2, text);
                    b.listener = new Button.ButtonListener(){

                        @Override
                        public void onClick() {
                            client.sendSettings();
                        }
                    };
                    buttons.add(b);

                    break;
                }
            }

            drawGUI(canvas);
            video.getHolder().unlockCanvasAndPost(canvas);

        }
        else
        {
            currentOperation = OFF;
        }

    }

    String getLeftText()
    {
        String text = (facePosition == FACE_LEFT) ? "*" : "";
        return "L" + text;
    }

    String getCenterText() {
        String text = (facePosition == FACE_CENTER) ? "*" : "";
        return "C" + text;
    }

    String getRightText()
    {
        String text = (facePosition == FACE_RIGHT) ? "*" : "";
        return "R" + text;
    }

    public void changeOperation() {
        getActivity().runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            changeOperationSync();
                                        }
                                    }
        );
    }

    public void drawGUI(Canvas canvas) {
        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);


        // DEBUG
   //     preview_x = 248;

        // erase background
        p.setColor(Color.DKGRAY);
        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);

        float dstX = canvas.getWidth() / 2;
        float dstY = canvas.getHeight() / 2;

        Matrix matrix = new Matrix();
        matrix.reset();
        matrix.postTranslate(-W / 2, -H / 2); // Centers source image

        float dstH;
        float dstW;
        float scale;
        dstW = canvas.getWidth();
        scale = dstW / H;
        matrix.postScale(scale, scale);
        matrix.postRotate(90);
        matrix.postTranslate(dstX, dstY);

//        Log.i("x", "drawGUI w=" + videoBitmap.getWidth() + " h=" + videoBitmap.getHeight());

        canvas.drawBitmap(videoBitmap,
                matrix,
                p);

        for (int i = 0; i < buttons.size(); i++)
        {
            buttons.get(i).draw(canvas);
        }

        for (int i = 0; i < texts.size(); i++)
        {
            texts.get(i).draw(canvas);
        }



    }





    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        int pointers = motionEvent.getPointerCount();
        //Log.i("FirstFragment", "motionEvent.getAction()=" + motionEvent.getAction());

        boolean needRedraw = false;
        for(int i = 0; i < buttons.size(); i++)
        {
            if(buttons.get(i).onTouch(motionEvent))
            {
                needRedraw = true;
                break;
            }
        }

        if(needRedraw)
        {
            Canvas canvas = video.getHolder().lockCanvas();
            drawGUI(canvas);
            video.getHolder().unlockCanvasAndPost(canvas);
        }

        return true;
    }


}
