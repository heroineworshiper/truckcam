package x.tracker;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.util.Log;
import android.view.MotionEvent;

public class Button {
    int centerX;
    int centerY;
    Rect rect = new Rect();
    Rect textSize = new Rect();
    String text = "";

    static final int NO_ARROW = 0;
    static final int UP = 1;
    static final int DOWN = 2;
    static final int LEFT = 3;
    static final int RIGHT = 4;
    int arrow = NO_ARROW;

    static final int MARGIN = 20;

    boolean isDown = false;
    boolean isHighlighted = false;
    ButtonListener listener = null;
    Thread repeater;


    interface ButtonListener
    {
        void onClick();
    }


//    Button(Rect rect, String text)
//    {
//        this.rect = rect;
//        this.text = text;
//    }

    Button(Rect rect, int arrow)
    {
        this.rect = rect;
        this.arrow = arrow;
    }

    Button(int centerX, int centerY, String text)
    {
        this.centerX = centerX;
        this.centerY = centerY;
        this.text = text;
        calculateRect();
    }

    // calculate arrow extents
    static Rect calculateRect(int centerX, int centerY, int w, int h) {
        Rect result = new Rect();
        result.left = centerX - w / 2;
        result.top = centerY - h / 2;
        result.right = result.left + w;
        result.bottom = result.top + h;
        return result;
    }

    // calculate button extents wkth text
    void calculateRect()
    {
        Rect size = calculateSize(text, textSize);
        rect.top = centerY - size.height() / 2;
        rect.bottom = rect.top + size.height();
        rect.left = centerX - size.width() / 2;
        rect.right = rect.left + size.width();


//        Log.i("Button", "textSize=" + textRect.top + " " + textRect.bottom + " " + textRect.left + " " + textRect.right);
//        Log.i("Button", "center=" + centerX + " " + centerY);
//        Log.i("Button", "rect=" + result.top + " " + result.bottom + " " + result.left + " " + result.right);
    }

    // return size
    static Rect calculateSize(String text, Rect textSize)
    {
        if(textSize == null)
        {
            textSize = new Rect();
        }

        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);
        p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
        p.setTextSize(FirstFragment.TEXT_SIZE);
        p.getTextBounds(text,
                0,
                text.length(),
                textSize);

        Rect size = new Rect();
        if(FirstFragment.landscape)
        {
            size.bottom = textSize.width() + MARGIN * 2;
            size.right = textSize.height() + MARGIN * 2;
        }
        else
        {
            size.bottom = textSize.height() + MARGIN * 2;
            size.right = textSize.width() + MARGIN * 2;
        }
        return size;
    }

    public boolean updateText(String text) {
        if(!this.text.equals(text)) {
            this.text = text;
            calculateRect();
            return true;
        }
        return false;
    }

    int getW()
    {
        return rect.width();
    }

    int getH()
    {
        return rect.height();
    }

    public void draw(Canvas canvas)
    {
//        Log.i("Button", "draw");
        Paint p = new Paint();

        p.setColor(Color.GREEN);

        // invert it if down
        if(isDown)
        {
            p.setStyle(Paint.Style.FILL);
        }
        else
        {
            p.setStyle(Paint.Style.STROKE);
        }
        p.setStrokeWidth(2);

        if(arrow != NO_ARROW) {
            final double[][] shape =
            {
                { -.5,  -.5 },
                { 0.0,  .5 },
                { .5,  -.5 },
                { -.5,  -.5 },
            };

            Path path = new Path();
            // Rotate shape & scale
            double angle = 0;
            double length = rect.height();
            double width = rect.width();
            switch(arrow)
            {
                case UP:
                    angle = -Math.PI / 2;
                    break;
                case DOWN:
                    angle = Math.PI / 2;
                    break;
                case LEFT:
                    angle = -Math.PI;
                    break;
                case RIGHT:
                    angle = 0;
                    break;
            }
            for(int i = 0; i < shape.length; i++)
            {

                double[] point = shape[i];
                double x1 = point[0] * length;
                double y1 = point[1] * width;
                double r = Math.sqrt(x1 * x1 + y1 * y1);
                double a = Math.atan2(y1, x1);
                a += angle;
                double x = (rect.left + rect.right) / 2;
                double y = (rect.top + rect.bottom) / 2;
                x1 = (float)(x + r * Math.cos(a));
                y1 = (float)(y + r * Math.sin(a));

                if(i == 0)
                    path.moveTo((float)x1, (float)y1);
                else
                    path.lineTo((float)x1, (float)y1);
            }

            canvas.drawPath(path, p);
        }
        else
        {

            canvas.drawRect(rect, p);

            if (isDown) {
                p.setColor(Color.BLACK);
            }

            if (!text.isEmpty()) {
                p.setStyle(Paint.Style.FILL);
                p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
                p.setTextSize(FirstFragment.TEXT_SIZE);

                if(FirstFragment.landscape) {
                    // make a temporary canvas for rotating the text
                    Bitmap temp2 = Bitmap.createBitmap(textSize.width(), textSize.height(), Bitmap.Config.ARGB_8888);
                    Canvas temp = new Canvas(temp2);
                    temp.drawText(text, 0, temp.getHeight() - textSize.bottom, p);

                    // rotate & draw it
                    Matrix matrix = new Matrix();
                    matrix.reset();
                    matrix.postRotate(90);
                    matrix.postTranslate(temp.getHeight() / 2 + (rect.left + rect.right) / 2,
                            -temp.getWidth() / 2 + (rect.top + rect.bottom) / 2);
                    canvas.drawBitmap(temp2,
                            matrix,
                            p);
                }
                else
                {
                    canvas.drawText(text, (rect.left + rect.right) / 2 - textSize.width() / 2,
                            rect.bottom - textSize.bottom - MARGIN,
                            p);

                }
            }
        }
    }

    public boolean onTouch(MotionEvent motionEvent) {
        switch(motionEvent.getAction())
        {
            case MotionEvent.ACTION_DOWN:
                if(motionEvent.getX() >= rect.left &&
                    motionEvent.getX() < rect.right &&
                    motionEvent.getY() >= rect.top &&
                    motionEvent.getY() < rect.bottom)
                {
                    isHighlighted = true;
                    isDown = true;
                    if(arrow != NO_ARROW)
                    {
                        if(listener != null)
                        {
                            listener.onClick();
                        }
                        startRepeater();
                    }
                    return true;
                }
                break;

            case MotionEvent.ACTION_MOVE:
                if(isHighlighted) {
                    // always active if it's an arrow
                    if(arrow == NO_ARROW) {
                        boolean newDown = false;

                        if (motionEvent.getX() >= rect.left &&
                                motionEvent.getX() < rect.right &&
                                motionEvent.getY() >= rect.top &&
                                motionEvent.getY() < rect.bottom) {
                            newDown = true;
                        }

                        if (newDown != isDown) {
                            isDown = newDown;
                            return true;
                        }
                    }
                }
                break;

            case MotionEvent.ACTION_UP:
                if(isHighlighted)
                {
                    isHighlighted = false;
                    if(isDown)
                    {
                        if(listener != null && arrow == NO_ARROW)
                        {
                            listener.onClick();
                        }
                        isDown = false;
                    }
                    if(arrow != NO_ARROW)
                    {
                        stopRepeater();
                    }
                    return true;
                }
                break;
            default:
                //Log.i("Button", "onTouch id=" + motionEvent.getAction());
                break;
        }

        return false;
    }

    private void startRepeater() {
        repeater = new Thread()
        {
            public void run()
            {
                int repeats = 0;
                boolean interrupted = false;
                while(!interrupted)
                {
                    try {
                        if(repeats == 0)
                        {
                            Thread.sleep(1000L);
                        }
                        else
                        {
                            Thread.sleep(150L);
                        }
                    } catch (InterruptedException e) {
                        interrupted = true;
                        e.printStackTrace();
                    }

                    if(!interrupted) {
                        repeats++;
                        if(listener != null)
                        {
                            listener.onClick();
                        }

                    }
                }
            }
        };
        repeater.start();
    }

    void stopRepeater()
    {
        repeater.interrupt();
        try {
            repeater.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
        repeater = null;
    }
}
