package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import androidx.annotation.ColorInt;

public class STEMperFiConstants {


    public static final @ColorInt int COLOR_RED = Color.rgb(250, 0, 0);
    public static final @ColorInt int COLOR_ORANGE =  Color.rgb(250, 165, 0);//Color.parseColor("orange");
    public static final @ColorInt int COLOR_YELLOW = Color.rgb(250,250,200);// Color.parseColor("yellow");
    public static final @ColorInt int COLOR_GREEN = Color.rgb(0,250,0);
    public static final @ColorInt int COLOR_BLUE = Color.rgb(0,0,250);
    public static final @ColorInt int COLOR_PURPLE = Color.rgb(128,0,128);

    public enum STATE {
        INTAKE,
        TRANSFER_START,
        TRANSFER_PINCH,
        TRANSFER_FLIP,
        DRIVE_TO_HUMAN,
        PLACE_PIXEL,
        HANG,
        INTAKE_PREP
    };

    public static final double BUCKET_INIT = 0.85;
    public static final double BUCKET_INTAKE = 0.77;
    public static final double BUCKET_SCORE = 0.5;

    public static final double ARM_INIT = 0.82;
    public static final double ARM_PICK_UP = 0.15;
    public static final double ARM_DROP = 0.91;
    public static final double ARM_DRIVE = 0.75;
    public static final double ARM_WALL = 0.38;

    public static final double GB_LED_OFF = 0;
    public static final double GB_LED_RED = 0.277;
    public static final double GB_LED_ORANGE = 0.333;
    public static final double GB_LED_YELLOW = 0.333;
    public static final double GB_LED_SAGE = 0.333;
    public static final double GB_LED_GREEN = 0.333;
    public static final double GB_LED_AZURE = 0.555;
    public static final double GB_LED_BLUE = 0.611;
    public static final double GB_LED_INDIGO = 0.666;
    public static final double GB_LED_VIOLET = 0.722;
    public static final double GB_LED_WHITE = 1.0;

}
