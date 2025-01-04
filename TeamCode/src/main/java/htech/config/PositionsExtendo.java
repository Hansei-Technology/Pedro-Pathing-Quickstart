package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionsExtendo {
    public static int ground = 0;
    public static int transfer = 0;
    public static int max = 380;
    public static int maxAuto = 345;
    public static int mid = 250;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;

    public static double freeSpeed = 0.4;
}
