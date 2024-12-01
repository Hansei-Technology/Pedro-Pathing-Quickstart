package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for intake subsystems.
 */
@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.558;
    public static double openedClaw = 0.4;

    // INTAKE ROTATION //
    public static double normalRotation = 0.275;
    public static double perpendicularRotation = 0.56;
    public static double flippedNormalRotation = 0.84;

    // INTAKE BAR //
    public static double groundPositionBar = 0.64;
    public static double wallPositionBar = 0.55;
    public static double transferPositionBar = 0.33; // maybe change this
    public static double collectPositionBar = 0.69; // collect position = ground ??
    public static double offsetBar = 0.025;

    // INTAKE JOINT //
    public static double groundPositionJoint = 0.55;
    public static double wallPickupPositionJoint = 0.40;
    public static double prepTransferPositionJoint = 0.23;
    public static double finalTransferPositionJoint = 0.19;
    public static double collectPositionJoint = 0.55;

}