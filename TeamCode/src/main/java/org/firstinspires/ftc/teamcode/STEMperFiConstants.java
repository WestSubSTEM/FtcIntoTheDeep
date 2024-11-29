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

    public static final double BUCKET_INIT = 0.66;
    public static final double BUCKET_INTAKE = 0.62;
    public static final double BUCKET_SCORE = 0.58;

    public static final double ARM_AIM = 0.5;
    public static final double ARM_INTAKE = 0.52;
    public static final double ARM_SCORE = 0.3;
    public static final double ARM_DRIVE = 0.4;
    public static final double ARM_WALL = 0.4;
    public static final double ARM_INIT = 0.33;

    public static final double PINCHER_CLOSE = 0.95;
    public static final double PINCHER_OPEN = 0.5;

    public static final double GB_LED_OFF = 0;
    public static final double GB_LED_RED = 0.28;
    public static final double GB_LED_ORANGE = 0.333;
    public static final double GB_LED_YELLOW = 0.388;
    public static final double GB_LED_SAGE = 0.444;
    public static final double GB_LED_GREEN = 0.5;
    public static final double GB_LED_AZURE = 0.555;
    public static final double GB_LED_BLUE = 0.611;
    public static final double GB_LED_INDIGO = 0.666;
    public static final double GB_LED_VIOLET = 0.722;
    public static final double GB_LED_WHITE = 1.0;

//    public static final double INTAKE_OFF = 0.5;
//    public static final double INTAKE_IN = -0.5;
//    public static final double INTAKE_OUT = 0.5;

    public static final int SCORE_BUCKET_FIRST = 1864;
    public static final int SCORE_BUCKET_SECOND = 3100;

    public static final int H_SCORE = 171;

    public static final long ATTACK_PINCHER_CLOSE_ms = 500;
    public static final long ATTACK_DRIVE_ms = 1_000;
    public static final long LIFT_DELAY_MS = 500;
}
