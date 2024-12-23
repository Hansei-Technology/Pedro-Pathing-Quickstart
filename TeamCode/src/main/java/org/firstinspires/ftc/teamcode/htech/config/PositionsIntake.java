package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for intake subsystems.
 */
@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.6;
    public static double openedClaw = 0.36;

    // INTAKE ROTATION //
//    public static double normalRotation = 0.275;
//    public static double perpendicularRotation = 0.56;
//    public static double flippedNormalRotation = 0.84;

    public static double normalRotation = 0.315;
    public static double perpendicularRotation = 0.6;
    public static double flippedNormalRotation = 0.88;

    public static double rotation30Deg = 0.095;
    public static double rotSpeed = 0.06  ;

    // INTAKE BAR //
    public static double groundPositionBar = 0.58; // over the samples
    public static double wallPositionBar = 0.52;
    public static double transferPositionBar = 0.245; // maybe change this
    public static double collectPositionBar = 0.63; // collect position = lower than ground
    public static double offsetBar = 0; //difference between the two bars

    // INTAKE JOINT //
    public static double groundPositionJoint = 0.56;
    public static double wallPickupPositionJoint = 0.42;
    public static double prepTransferPositionJoint = 0.25;
    public static double finalTransferPositionJoint = 0.2;
    public static double collectPositionJoint = 0.56;

}