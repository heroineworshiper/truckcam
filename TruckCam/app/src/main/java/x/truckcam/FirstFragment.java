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
    static FirstFragment first;
    SurfaceView video;
    Bitmap videoBitmap;
    Canvas videoCanvas;
    static ClientThread client;

// bits for efficientdet
// max faces + max bodies
    static final int MAX_BOXES = 10;
    static int animals;
// raw, interleaved x & y coords of rectangles
    static int[] coords = new int[MAX_BOXES * 4];
// names of animals
    static String[] names = new String[MAX_BOXES];
//    static int faces;
//    static int[] face_coords = new int[MAX_BOXES * 4];
    static int nearest_box = 0;


// bits for posenet
// defined by the pose model
    static final int BODY_PARTS = 17;
    static final int INVALID_BODY_PART = 0x7fff;
    static final int MAX_ANIMALS = 6;
// input coordinates
    static int[] keypoints = new int[MAX_ANIMALS * BODY_PARTS * 2];
// screen coordinates
    static int[] keypoint_cache = new int[MAX_ANIMALS * BODY_PARTS * 2];
// class BodyPart
    static final int NOSE = 0;
    static final int LEFT_EYE = 1;
    static final int RIGHT_EYE = 2;
    static final int LEFT_EAR = 3;
    static final int RIGHT_EAR = 4;
    static final int LEFT_SHOULDER = 5;
    static final int RIGHT_SHOULDER = 6;
    static final int LEFT_ELBOW = 7;
    static final int RIGHT_ELBOW = 8;
    static final int LEFT_WRIST = 9;
    static final int RIGHT_WRIST = 10;
    static final int LEFT_HIP = 11;
    static final int RIGHT_HIP = 12;
    static final int LEFT_KNEE = 13;
    static final int RIGHT_KNEE = 14;
    static final int LEFT_ANKLE = 15;
    static final int RIGHT_ANKLE = 16;




// in case frames come in faster than we can draw them
    static boolean busy = false;

// size of the preview video.  Changes based on the input frames
    static int H = 320;
    static int W = 240;

// center of video
    static float dstX;
    static float dstY;
// size of video
    static float dstH;
    static float dstW;

    static final int OFF = -1;
    static final int IDLE  = 0;
    static final int TRACKING = 1;
    static int currentOperation = OFF;
    static int nextOperation = OFF;
    static boolean landscape = true;
    static boolean prevLandscape = true;

    static float fps = 0;
    static int framesCaptured = 0;
    static boolean isCapturing = false;

// error codes
    final int SERVO_ERROR = 0x80;
    final int CAM_ERROR_MASK = 0x0f;
    final int VIDEO_DEVICE_ERROR = 1;
    final int VIDEO_CONFIG_ERROR = 2;
    final int CAM_ENUM_ERROR = 3;
    final int CAM_STARTING_ERROR = 4;
    final int CAM_SELECT_ERROR = 5;
    final int CAM_BUFFER_ERROR = 6;

    int errors;

    float accelX = 0;
    float accelY = 0;
    final float ACCEL_BANDWIDTH = (float)0.2;

    Vector<Button> buttons = new Vector();
    Vector<Text> texts = new Vector();
    Button activateButton;
    Button settingsButton;
    Button captureButton;
    Text videoError;
    Text servoError;
    Text fpsText;

    static final int MARGIN = 20;
    static final int ARROW_MARGIN = 20;
    static final int TEXT_SIZE = 40;
    static final String settingsText = "SEND CONFIG";
    long startTime;

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



        first = this;
        MainActivity.init();

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

// must know W, H before creating the drawing objects
                    if(videoBitmap == null)
                    {
                        W = bitmap.getWidth();
                        H = bitmap.getHeight();
                        videoBitmap = Bitmap.createBitmap(W, H, Bitmap.Config.ARGB_8888);
                        videoCanvas = new Canvas();
                        videoCanvas.setBitmap(videoBitmap);
                    }

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
        if((errors & CAM_ERROR_MASK) != 0)
        {
            needRedraw |= videoError.setHidden(false);
            if((errors & CAM_ERROR_MASK) == VIDEO_DEVICE_ERROR)
                needRedraw |= videoError.updateText("/DEV/VIDEO* NOT FOUND");
            else
            if((errors & CAM_ERROR_MASK) == VIDEO_CONFIG_ERROR)
                needRedraw |= videoError.updateText("VIDEO CONFIG FAILED");
            else
            if((errors & CAM_ERROR_MASK) == CAM_ENUM_ERROR)
                needRedraw |= videoError.updateText("VIDEO USB NOT FOUND");
            else
            if((errors & CAM_ERROR_MASK) == CAM_STARTING_ERROR)
                needRedraw |= videoError.updateText("VIDEO STARTING");
            else
            if((errors & CAM_ERROR_MASK) == CAM_SELECT_ERROR)
                needRedraw |= videoError.updateText("VIDEO SELECT FAILED");
            else
            if((errors & CAM_ERROR_MASK) == CAM_BUFFER_ERROR)
                needRedraw |= videoError.updateText("VIDEO BUFFER FAILED");
        }
        else
        {
            needRedraw |= videoError.setHidden(true);
        }

        needRedraw |= servoError.setHidden((errors & SERVO_ERROR) == 0);
        return needRedraw;
    }

    public void updateValues() {
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                boolean needRedraw = false;

                needRedraw |= updateErrors();

                if(settingsButton != null)
                {
// show the * for a minimum time
                    long currentTime = System.currentTimeMillis();
                    long diffTime = currentTime - startTime;
                    if((diffTime >= 500 && 
                        settingsButton.text.contains("*") && !ClientThread.needSettings) ||
                        (!settingsButton.text.contains("*") && ClientThread.needSettings))
                        needRedraw = true;
                }
                
                if(captureButton != null)
                {
                    String captureText = "RECORD";
                    if(isCapturing) captureText += "*";
                    if(captureText != captureButton.text)
                    {
                        captureButton.updateText(captureText);
                        needRedraw = true;
                    }
                }

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
        if(currentOperation == nextOperation) return;

        Canvas canvas = video.getHolder().lockCanvas();

        if (canvas != null) {
            buttons.clear();
            texts.clear();

            activateButton = null;
            settingsButton = null;
            captureButton = null;


            int x, y;
            Rect size = Text.calculateSize("X");
            x = canvas.getWidth() - size.height();
            y = canvas.getHeight() / 2;

// landscape mode
            fpsText = new Text(canvas.getWidth() - size.height(), 
                0, "FPS: ");
            fpsText.color = Color.GREEN;
            texts.add(fpsText);


            servoError = new Text(x, y, "SERVO NOT FOUND");
            servoError.color = Color.RED;
            x -= size.height() + MARGIN;
            videoError = new Text(x, y, "UNDEFINED VIDEO ERROR");
            videoError.color = Color.RED;
            x -= size.height() + MARGIN;


            texts.add(videoError);
            texts.add(servoError);
            updateErrors();

            switch(nextOperation)
            {
                case IDLE:
                case TRACKING: {
// Always a tracking/abort button
                    String text;
                    if(nextOperation == IDLE) {
                        text = "ACTIVATE SERVO";
                    }
                    else{
                        text = "ABORT";
                    }
// add * to line it up with the other buttons
                    size = Button.calculateSize(text + "*", null);
                    x = canvas.getWidth() - size.width() - MARGIN;
                    y = MARGIN + size.height() / 2;


                    activateButton = new Button(x, y, text);
                    activateButton.listener = new Button.ButtonListener() {
                        @Override
                        public void onClick() {
                            Log.i("FirstFragment", "ACTIVATE SERVO");
                            if(nextOperation == IDLE) {
                                client.sendCommand(' ');
                            }
                            else{
                                client.sendCommand('q');
                            }
                        }
                    };
                    buttons.add(activateButton);

                    String captureText = "RECORD";
                    if(isCapturing) captureText += "*";
                    x = canvas.getWidth() / 2;
                    size = Button.calculateSize("RECORD*", null);
                    captureButton = new Button(x, 
                        MARGIN + size.height() / 2, 
                        captureText);
                    captureButton.listener = new Button.ButtonListener(){

                        @Override
                        public void onClick() {
                            client.sendCommand('c');
                        }
                    };
                    buttons.add(captureButton);

                    if(nextOperation == IDLE)
                    {
                        x = MARGIN + size.width() / 2;
                        size = Button.calculateSize(settingsText + "*", null);
                        settingsButton = new Button(x, 
                            MARGIN + size.height() / 2, 
                            settingsText);
                        settingsButton.listener = new Button.ButtonListener(){

                            @Override
                            public void onClick() {
                                startTime = System.currentTimeMillis();
                                client.sendSettings();
                            }
                        };
                        buttons.add(settingsButton);


                    }

                    break;
                }
            }

            drawGUI(canvas);
            video.getHolder().unlockCanvasAndPost(canvas);
            currentOperation = nextOperation;
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


    static public void drawBodyPart(int animal,
        int bodyPart, 
        Canvas c, 
        Paint p)
    {
        int radius;
        if(currentOperation != TRACKING)
            radius = 4;
        else
            radius = 8;
        int offset = animal * BODY_PARTS * 2 + bodyPart * 2;
        int x = keypoints[offset];
        int y = keypoints[offset + 1];
        if(x != INVALID_BODY_PART || y != INVALID_BODY_PART) 
        {
            int x2 = 0;
            int y2 = 0;
// scale to screen positions
            if(landscape)
            {
                x2 = (int)(dstX + dstW / 2 - y * dstW / H);
                y2 = (int)(dstY - dstH / 2 + x * dstH / W);
            }
            else
            {
                x2 = (int)(dstX - dstW / 2 + x * dstW / H);
// crop & scale portrait height
                float portraitScale = (float)(540.0 / 640);
                y2 = (int)(dstY - dstH * portraitScale / 2 + y * dstH / W);
            }
Log.i("x", "drawBodyPart x=" + x + " y=" + y + " x2=" + x2 + " y2=" + y2);

            p.setStrokeWidth(1);
            p.setStyle(Paint.Style.FILL);
            c.drawCircle(x2, y2, radius, p);
// store the screen coords for later
            keypoint_cache[offset] = x2;
            keypoint_cache[offset + 1] = y2;
        }
        else
        {
            keypoint_cache[offset] = INVALID_BODY_PART;
            keypoint_cache[offset + 1] = INVALID_BODY_PART;
        }
    }


    static public void joinBodyParts(Canvas c, 
        Paint p,
        int animal,
        int bodyPart1, 
        int bodyPart2)
    {
        int offset = animal * BODY_PARTS * 2;

        int x1 = keypoint_cache[offset + bodyPart1 * 2];
        int y1 = keypoint_cache[offset + bodyPart1 * 2 + 1];
        int x2 = keypoint_cache[offset + bodyPart2 * 2];
        int y2 = keypoint_cache[offset + bodyPart2 * 2 + 1];

        if((x1 != INVALID_BODY_PART || y1 != INVALID_BODY_PART) &&
            (x2 != INVALID_BODY_PART || y2 != INVALID_BODY_PART))
        {
            p.setStyle(Paint.Style.STROKE);
            p.setStrokeWidth(4);
            c.drawLine(x1, y1, x2, y2, p);
        }
    }

    public void drawGUI(Canvas canvas) 
    {
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
        dstH = dstW * W / H;
        matrix.postScale(scale, scale);
        matrix.postRotate(90);
        matrix.postTranslate(dstX, dstY);

//Log.i("x", "drawGUI");

//        Log.i("x", "drawGUI w=" + videoBitmap.getWidth() + " h=" + videoBitmap.getHeight());

        if(videoBitmap != null)
            canvas.drawBitmap(videoBitmap,
                    matrix,
                    p);

// draw bounding boxes
        if(ClientThread.serverIsTruck)
        {
            p.setStyle(Paint.Style.STROKE);
            p.setStrokeWidth(4);
            for(int j = 0; j < animals; j++)
            {
                int x1 = coords[j * 4 + 0];
                int y1 = coords[j * 4 + 1];
                int x2 = x1 + coords[j * 4 + 2];
                int y2 = y1 + coords[j * 4 + 3];
    //Log.i("x", "drawGUI " + x1 + " " + y1 + " "  + x2 + " "  + y2);

                if(j == nearest_box) 
                    p.setColor(0xffffff00);
                else
                    p.setColor(0xff00ff00);
                Rect screenCoords = serverToScreen(x1, y1, x2, y2);
                canvas.drawRect(screenCoords, p);

// draw name
                if(names[j].length() > 0)
                {
                    Rect textSize = Text.calculateSize(names[j]);
                    // landscape coords
                    Text nameText = new Text(screenCoords.right - textSize.height(),
                        screenCoords.top,
                        names[j]);
                    nameText.color = p.getColor();
                    nameText.draw(canvas);
                }
            }

// faces
//             p.setColor(0xff00ffff);
//             p.setStrokeWidth(2);
//             for(int j = 0; j < faces; j++)
//             {
//                 int x1 = face_coords[j * 4 + 0];
//                 int y1 = face_coords[j * 4 + 1];
//                 int x2 = x1 + face_coords[j * 4 + 2];
//                 int y2 = y1 + face_coords[j * 4 + 3];
//     //Log.i("x", "drawGUI " + x1 + " " + y1 + " "  + x2 + " "  + y2);
// 
//                 Rect screenCoords = serverToScreen(x1, y1, x2, y2);
//                 canvas.drawRect(screenCoords, p);
//             }
        }
        else
        {
// draw poses
            p.setStyle(Paint.Style.STROKE);
            p.setColor(0xff00ff00);
            p.setStrokeWidth(4);
Log.i("x", "drawGUI 1");
            for(int j = 0; j < animals; j++)
            {
                for(int i = 0; i < FirstFragment.BODY_PARTS; i++)
                {
                    drawBodyPart(j, 
                        i, 
                        canvas, 
                        p);
                }
                joinBodyParts(canvas, p, j, LEFT_EYE, NOSE);
                joinBodyParts(canvas, p, j, LEFT_EYE, LEFT_EAR);
                joinBodyParts(canvas, p, j, RIGHT_EYE, NOSE);
                joinBodyParts(canvas, p, j, RIGHT_EYE, RIGHT_EAR);
                joinBodyParts(canvas, p, j, LEFT_SHOULDER, RIGHT_SHOULDER);
                joinBodyParts(canvas, p, j, LEFT_SHOULDER, LEFT_ELBOW);
                joinBodyParts(canvas, p, j, LEFT_ELBOW, LEFT_WRIST);
                joinBodyParts(canvas, p, j, RIGHT_SHOULDER, RIGHT_ELBOW);
                joinBodyParts(canvas, p, j, RIGHT_ELBOW, RIGHT_WRIST);
                joinBodyParts(canvas, p, j, LEFT_HIP, RIGHT_HIP);
                joinBodyParts(canvas, p, j, LEFT_HIP, LEFT_KNEE);
                joinBodyParts(canvas, p, j, LEFT_KNEE, LEFT_ANKLE);
                joinBodyParts(canvas, p, j, RIGHT_HIP, RIGHT_KNEE);
                joinBodyParts(canvas, p, j, RIGHT_KNEE, RIGHT_ANKLE);

                p.setStyle(Paint.Style.STROKE);
                p.setStrokeWidth(4);

                int offset = j * BODY_PARTS * 2;
                int noseX = keypoint_cache[offset + NOSE * 2];
                int noseY = keypoint_cache[offset + NOSE * 2 + 1];
                int leftShoulderX = keypoint_cache[offset + LEFT_SHOULDER * 2];
                int leftShoulderY = keypoint_cache[offset + LEFT_SHOULDER * 2 + 1];
                int rightShoulderX = keypoint_cache[offset + RIGHT_SHOULDER * 2];
                int rightShoulderY = keypoint_cache[offset + RIGHT_SHOULDER * 2 + 1];
                int leftHipX = keypoint_cache[offset + LEFT_HIP * 2];
                int leftHipY = keypoint_cache[offset + LEFT_HIP * 2 + 1];
                int rightHipX = keypoint_cache[offset + RIGHT_HIP * 2];
                int rightHipY = keypoint_cache[offset + RIGHT_HIP * 2 + 1];
                int neckX = INVALID_BODY_PART;
                int neckY = INVALID_BODY_PART;
                int buttX = INVALID_BODY_PART;
                int buttY = INVALID_BODY_PART;

                if(leftShoulderX != INVALID_BODY_PART &&
                    leftShoulderY != INVALID_BODY_PART &&
                    rightShoulderX != INVALID_BODY_PART &&
                    rightShoulderY != INVALID_BODY_PART)
                {
                    neckX = (leftShoulderX + rightShoulderX) / 2;
                    neckY = (leftShoulderY + rightShoulderY) / 2;
                }

                if(leftHipX != INVALID_BODY_PART &&
                    leftHipY != INVALID_BODY_PART &&
                    rightHipX != INVALID_BODY_PART &&
                    rightHipY != INVALID_BODY_PART)
                {
                    buttX = (leftHipX + rightHipX) / 2;
                    buttY = (leftHipY + rightHipY) / 2;
                }

                if(noseX != INVALID_BODY_PART &&
                    noseY != INVALID_BODY_PART &&
                    neckX != INVALID_BODY_PART &&
                    neckY != INVALID_BODY_PART)
                {
                    canvas.drawLine(noseX, noseY, neckX, neckY, p);
                }

                if(buttX != INVALID_BODY_PART &&
                    buttY != INVALID_BODY_PART &&
                    neckX != INVALID_BODY_PART &&
                    neckY != INVALID_BODY_PART)
                {
                    canvas.drawLine(buttX, buttY, neckX, neckY, p);
                }

            }
        }

        StringBuilder sb = new StringBuilder();
        Formatter formatter = new Formatter(sb);
//Log.i("x", "drawGUI fpsText=" + fpsText);
        if(fpsText != null) fpsText.updateText("FPS: " + formatter.format("%.02f", fps));

// update settings button
        if(settingsButton != null) 
        {
            if(ClientThread.needSettings)
                settingsButton.updateText(settingsText + "*");
            else
                settingsButton.updateText(settingsText);
        }

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
