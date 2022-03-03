package x.tracker;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.Typeface;

public class Text {
    Rect rect = new Rect();
    String text = "";
    int color = Color.GREEN;
    boolean hidden = false;
    int x, y;

    Text(int x, int y, String text)
    {
        this.text = text;
        this.x = x;
        this.y = y;
        calculateRect();

    }

    // return size not rotated for landscape mode
    static Rect calculateSize(String text)
    {
        Paint p = new Paint();
        Rect text_size = new Rect();
        p.setStyle(Paint.Style.FILL);
        p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
        p.setTextSize(FirstFragment.TEXT_SIZE);
        p.getTextBounds(text,
                0,
                text.length(),
                text_size);
        return text_size;
    }

    void calculateRect()
    {
        Rect temp = calculateSize(text);
        if(FirstFragment.landscape) {
            rect.left = x - temp.height() / 2;
            rect.top = y;
            rect.right = rect.left + temp.height();
            rect.bottom = rect.top + temp.width();
        }
        else
        {
            rect.left = x;
            rect.top = y - temp.height() / 2;
            rect.right = rect.left + temp.width();
            rect.bottom = rect.top + temp.height();
        }
    }

    int getW()
    {
        return rect.width();
    }

    int getH()
    {
        return rect.height();
    }

    public void draw(Canvas canvas) {
        if(hidden)
        {
            return;
        }

        Paint p = new Paint();
        p.setColor(color);
        p.setStyle(Paint.Style.FILL);
        p.setTypeface(Typeface.create("SansSerif", Typeface.BOLD));
        p.setTextSize(FirstFragment.TEXT_SIZE);
        Rect text_size = calculateSize(text);

        if(FirstFragment.landscape) {
            // make a temporary canvas for rotating the text
            Bitmap temp2 = Bitmap.createBitmap(text_size.width(), text_size.height(), Bitmap.Config.ARGB_8888);
            Canvas temp = new Canvas(temp2);
            temp.drawText(text, 0, temp.getHeight() - text_size.bottom, p);

            // rotate & draw it
            Matrix matrix = new Matrix();
            matrix.reset();
            //matrix.postTranslate(-temp.getWidth() / 2,
            //        -temp.getHeight() / 2); // Centers image
            matrix.postRotate(90);
            matrix.postTranslate(temp.getHeight() / 2 + (rect.left + rect.right) / 2,
                    -temp.getWidth() / 2 + (rect.top + rect.bottom) / 2);
            canvas.drawBitmap(temp2,
                    matrix,
                    p);
        }
        else
        {
            canvas.drawText(text, rect.left, rect.bottom - text_size.bottom, p);
        }
    }


    public boolean updateText(String s) {
        if(!this.text.equals(s)) {
            this.text = s;
            calculateRect();
            return true;
        }
        return false;
    }

    boolean setHidden(boolean value)
    {
        if(this.hidden != value)
        {
            this.hidden = value;
            return true;
        }

        return false;
    }
}
