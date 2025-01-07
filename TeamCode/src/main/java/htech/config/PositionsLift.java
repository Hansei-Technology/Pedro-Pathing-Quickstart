package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PositionsLift {
    public static int ground = 0;
    public static int park = 300;
    public static int transfer = 0;
    public static int lowChamber = 300;
    public static int highChamber = 600;
    public static int lowBasket = 600;
    public static int highBasket = 1450;
    public static int magic = 380; //this is the position just under the high chamber for scoring the specimen

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
}
