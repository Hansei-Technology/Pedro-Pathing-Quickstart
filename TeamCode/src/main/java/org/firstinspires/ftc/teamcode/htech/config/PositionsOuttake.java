package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for outtake subsystems.
 */
@Config
public abstract class PositionsOuttake {

    // OUTTAKE CLAW 
    public static double closedClaw = 0.48;
    public static double openedClaw = 0.34;

    // OUTTAKE BAR 
//    public static double transfer = 0.5;
//    public static double specimen = 0.5;
//    public static double score = 0.5;

    // OUTTAKE JOINT

    public static double jointSpecimenLeft = 0.565;
    public static double jointSpecimenRight = 0.215;
    public static double jointTransferLeft = 0.67;
    public static double jointTransferRight = 0.33;
    public static double jointBasketLeft = 0.21;
    public static double jointBasketRight = 0.44;
    public static double jointDropLeft = 0.06;
    public static double jointDropRight = 0.39;

    public static double jointRotation90 = -0.155;
}