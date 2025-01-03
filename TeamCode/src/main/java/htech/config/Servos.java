package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing Servo assigned slots specified in the Control Hub.
 */
@Config
public abstract class Servos {
    public static String outtakeLeft = "s0e";
    public static String outtakeRight = "s3e";
//    public static String outtakeJointLeft = "";
//    public static String outtakeJointRight = "";
    public static String outtakeClaw = "s1e";

    public static String intakeBarServoLeft = "s2";
    public static String intakeBarServoRight = "s5";

    public static String intakeJointServo = "s3";
    public static String intakeRotationServo = "s4";
    public static String intakeClawServo = "s1";


    public static String hangLeftServo = "s4e";
    public static String hangRightServo = "s5e";
}
