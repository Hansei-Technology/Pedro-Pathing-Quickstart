package org.firstinspires.ftc.teamcode.htech.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.htech.SpecimenAuto;

/**
 * Class used for storing the various subsystem dependent values, like speed.
 */
@Config
public abstract class RobotSettings {


    public static double speed = 1;
    public static double rotationSpeed = 0.9;

    public static double hangPower = 1;

    //timers are in milliseconds
    public static int timeToCloseClaw = 150;
    public static int timeDown_Transfer = 650;
    public static int timeWall_Transfer = 550;
    public static int timeReady_Transfer = 600;
    public static int timeToCatch = 180;  //150
    public static int timeWaitingToCatch = 300;
    public static int timeToCloseOuttake = 150;
    public static int timeFailedToCloseLift = 1000;

    public static int timeToCollect = 300;
    public static int timeToCollectGoingDown = 175;
    public static int timeToCollectGoingDownFast = 400;
    public static int timeToCollectGoingUp = 150;
    public static int timeToCollectFast = 200;

    public static int time_to_specimen = 200;
    public static int time_to_spit = 350;

}
