package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing the various subsystem dependent values, like speed.
 */
@Config
public abstract class RobotSettings {
    public static double speed = 1;
    public static double rotationSpeed = 0.9;

    public static double hangPower = 1;

    //timers are in milliseconds
    public static int timeDown_Transfer = 0;
    public static int timeWall_Transfer = 450;
    public static int timeReady_Transfer = 600;
    public static int timeToDropElement = 150;
    public static int timeToCloseOuttake = 150;

    public static int timeToCollect = 150;
    public static int timeToCollectGoingDown = 125;
    public static int timeToCollectGoingUp = 100;
}
