package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for intake subsystems.
 */
@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.37;
    public static double openedClaw = 0.5;

    // INTAKE ROTATION //
    public static double normalRotation = 0.275;
    public static double perpendicularRotation = 0.56;
    public static double flippedNormalRotation = 0.835;

    // INTAKE BAR //
    public static double groundPositionBar = 0.61;
    public static double wallPositionBar = 0.53;
    public static double transferPositionBar = 0.27; // maybe change this
    public static double collectPositionBar = 0.643; // collect position = ground ??

    // INTAKE JOINT //
    public static double groundPositionJoint = 0.65;
    public static double wallPickupPositionJoint = 0.47;
    public static double prepTransferPositionJoint = 0.345;
    public static double finalTransferPositionJoint = 0.30;
    public static double collectPositionJoint = 0.65;

}