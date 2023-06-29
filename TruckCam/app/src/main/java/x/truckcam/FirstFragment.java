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
import java.util.Formatter;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.fragment.NavHostFragment;
import x.truckcam.databinding.FragmentFirstBinding;

import static android.content.Context.MODE_APPEND;
import static android.content.Context.TELECOM_SERVICE;

public class FirstFragment extends Fragment implements View.OnTouchListener {

    private FragmentFirstBinding binding;
    SurfaceView video;
    Bitmap videoBitmap;
    Canvas videoCanvas;
    ClientThread client;

// max faces + max bodies
    static final int MAX_BOXES = 10;
    static int animals;
// raw, interleaved x & y coords of rectangles
    static int[] coords = new int[MAX_BOXES * 4];
// names of animals
    static String[] names = new String[MAX_BOXES];
    
// in case frames come in faster than we can draw them
    static boolean busy = false;

    // size of the cropped preview video
    static final int W = 640;
    static final int H = 360;

    static float dstX;
    static float dstY;
    static float dstH;
    static float dstW;

    final int OFF = -1;
    final int IDLE  = 0;
    final int TRACKING = 1;
    int currentOperation = OFF;
    int prevOperation = OFF;

    static float fps = 0;

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
    Text videoDeviceError;
    Text videoBufferError;
    Text servoError;
    Text fpsText;

    static final int MARGIN = 40;
    static final int ARROW_MARGIN = 20;
    static final int TEXT_SIZE = 40;

    @Override
    public View onCreateView(
            LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState
    ) {
// Always draw synthetic landscape widgets to get more space
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

    public void drawVideo(Bitmap bitmap) {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Canvas canvas = video.getHolder().lockCanvas();

                if(canvas != null) {
                    Paint p = new Paint();
// must convert to a software bitmap for draw()
                    Bitmap softBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, false);
                    videoCanvas.drawBitmap(softBitmap, 0, 0, p);


                    drawGUI(canvas);

                    video.getHolder().unlockCanvasAndPost(canvas);
                }
                busy = false;
            }
        });
    }

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



        Canvas canvas = video.getHolder().lockCanvas();

        if (canvas != null) {
            int x, y;
            Rect size = Text.calculateSize("X");
            x = canvas.getWidth() / 4;
            y = canvas.getHeight() / 2;

// landscape mode
            fpsText = new Text(canvas.getWidth() - size.height(), 
                0, "FPS: ");
            fpsText.color = Color.GREEN;
            texts.add(fpsText);


            videoDeviceError = new Text(x, y, "VIDEO DEVICE NOT FOUND");
            videoDeviceError.color = Color.RED;
            x += canvas.getWidth() / 4;
            videoBufferError = new Text(x, y, "VIDEO CAPTURE FAILED");
            videoBufferError.color = Color.RED;
            x += canvas.getWidth() / 4;

            servoError = new Text(x, y, "SERVO DEVICE NOT FOUND");
            servoError.color = Color.RED;

            texts.add(videoDeviceError);
            texts.add(videoBufferError);
            texts.add(servoError);
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
                    x = canvas.getWidth() - size.width() * 2 - MARGIN;
                    y = MARGIN + size.height() / 2;


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


    public void changeOperation() {
        getActivity().runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            changeOperationSync();
                                        }
                                    }
        );
    }

// convert the coords to a Rect with screen dimensions
    public Rect serverToScreen(int x1, int y1, int x2, int y2)
    {
        int x3, y3, x4, y4;
// scale & transpose to screen positions in landscape
        x3 = (int)(dstX + dstW / 2 - y2 * dstW / H);
        x4 = (int)(dstX + dstW / 2 - y1 * dstW / H);
        y3 = (int)(dstY - dstH / 2 + x1 * dstH / W);
        y4 = (int)(dstY - dstH / 2 + x2 * dstH / W);
//Log.i("x", "serverToScreen " + x3 + " " + y3 + " "  + x4 + " "  + y4);
        return new Rect(x3, y3, x4, y4);
    }

    public void drawGUI(Canvas canvas) {
        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);


        // DEBUG
   //     preview_x = 248;

        // erase background
        p.setColor(Color.DKGRAY);
        canvas.drawRect(new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), p);

        dstX = canvas.getWidth() / 2;
        dstY = canvas.getHeight() / 2;

        Matrix matrix = new Matrix();
        matrix.reset();
        matrix.postTranslate(-W / 2, -H / 2); // Centers source image

        float scale;
        dstW = canvas.getWidth();
        scale = dstW / H;
        dstH = dstW * 16 / 9;
        matrix.postScale(scale, scale);
        matrix.postRotate(90);
        matrix.postTranslate(dstX, dstY);

//Log.i("x", "drawGUI");

//        Log.i("x", "drawGUI w=" + videoBitmap.getWidth() + " h=" + videoBitmap.getHeight());

        canvas.drawBitmap(videoBitmap,
                matrix,
                p);

// draw bounding boxes
        p.setStyle(Paint.Style.STROKE);
        p.setColor(0xff00ff00);
        p.setStrokeWidth(4);
        for(int j = 0; j < animals; j++)
        {
            int x1 = coords[j * 4 + 0] / 2;
            int y1 = coords[j * 4 + 1] / 2;
            int x2 = coords[j * 4 + 2] / 2;
            int y2 = coords[j * 4 + 3] / 2;
//Log.i("x", "drawGUI " + x1 + " " + y1 + " "  + x2 + " "  + y2);

            Rect screenCoords = serverToScreen(x1, y1, x2, y2);
            canvas.drawRect(screenCoords, p);

            if(names[j].length() > 0)
            {
                Text nameText = new Text(screenCoords.left, 
                    screenCoords.bottom, 
                    names[j]);
                nameText.draw(canvas);
            }
        }

        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb);
        if(fpsText != null) fpsText.updateText("FPS: " + formatter.format("%.02f", fps));

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
