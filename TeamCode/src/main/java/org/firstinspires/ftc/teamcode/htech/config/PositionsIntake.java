package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for intake subsystems.
 */
@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.7;
    public static double openedClaw = 0.3;

    // INTAKE ROTATION //
    public static double normalRotation = 0.275;
    public static double perpendicularRotation = 0.56;
    public static double flippedNormalRotation = 0.84;

    // INTAKE BAR //
    public static double groundPositionBar = 0.66;
    public static double wallPositionBar = 0.5;
    public static double transferPositionBar = 0.3; // maybe change this
    public static double collectPositionBar = 0.69; // collect position = ground ??

    // INTAKE JOINT //
    public static double groundPositionJoint = 0.48;
    public static double wallPickupPositionJoint = 0.33;
    public static double prepTransferPositionJoint = 0.16;
    public static double finalTransferPositionJoint = 0.12;
    public static double collectPositionJoint = 0.48;

}