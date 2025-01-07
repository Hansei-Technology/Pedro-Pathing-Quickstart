package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing DcMotorEx assigned slots specified in the Control Hub.
 */
@Config
public abstract class Motors {
    public static String leftFrontMotor = "m1";
    public static String rightFrontMotor = "m2";
    public static String leftRearMotor = "m0";
    public static String rightRearMotor = "m3";

    public static String lift1 = "m2e";
    public static String lift2 = "m3e";
    public static String extendo = "m1e";
    public static String liftEncoder = "m1";
}
