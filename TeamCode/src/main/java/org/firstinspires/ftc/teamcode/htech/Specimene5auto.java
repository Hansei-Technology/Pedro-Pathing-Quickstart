package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.PositionsLift;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.htech.subsystem.ChassisMovement;
import org.firstinspires.ftc.teamcode.htech.subsystem.ExtendoSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.LiftSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.RobotSystems;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name = "[AUTO] Specimen", group = "HTECH")
public class Specimene5auto extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;

    private Follower follower;
    private Path goToPreload;
    private PathChain goToCollectingPose;
    private PathChain goToWall;
    private PathChain goToScore1;
    private PathChain goToScore2;
    private PathChain goToScore3;
    private PathChain goToScore4;
    private PathChain goToPark;
    private PathChain goToSample1;
    private PathChain goToSample2;
    private PathChain goToSample3;
    private PathChain goToDrop;

    private enum STATES {
        MOVING,
        WAITING,
        TRANSFERING,
        PRELOAD,
        PLACING_PRELOAD,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        SAFE1,
        SAFE2,
        SAFE3,
        PARK,
        PARKED,
        BACK_TO_WALL,
        WALL,
        COLLECTING,
        SCORE,
        CHECKPOINT,
        WAIT_CLESTE,
        WAIT_TRANSFER

    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;

    public static double maxSpeed = 0.9;
    public static double slowSpeed = 0.65;
    public static double preloadSpeed = 0.6;
    public static double mediumSpeed = 0.8;
    //public VoltageSensor voltageSensor;
    //public double voltage;
    boolean firstTime = true;

    public enum SCORING_STATES{
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3,
        IDLE,
        SPECIMEN4
    }

    public SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;

    public static int timeToPreload = 0;
    public static int timeToSample = 200;
    public static int timeToCollect = 100;
    public static int timeToCollect2 = 1000;
    public static int timeToCollect3 = 1000;
    public static int time_to_transfer = 1000;
    public static int time_to_lift = 650;
    public static int time_to_drop = 800;
    public static int time_to_extend = 150;

    //start pose 135, 83

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -26.5, PRELOAD_Y = 0, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = -10, SAFE1_Y = 20;
    public static double SCORE1_X = -28.5, SCORE1_Y = -6.5;
    public static double SCORE2_X = -28.5, SCORE2_Y = -8;
    public static double SCORE3_X = -28.5, SCORE3_Y = -9;
    public static double SCORE4_X = -28.5, SCORE4_Y = -10.5;

    public static double SPECIMEN_X = -9.5, SPECIMEN_Y = 30, SPECIMEN_ANGLE = 0; //pozitie specimen pe perete

    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 5;
    public static double SAFE_SPECIMEN2_X = -20, SAFE_SPECIMEN2_Y = SPECIMEN_Y;

    public static double PARK_X = -10, PARK_Y = 30, PARK_ANGLE = 80;
    public static double SAMPLE1_HEADING = 200, SAMPLE2_HEADING = 180, SAMPLE3_HEADING = 165, DROP_ANGLE = 75;
    public static double COLLECTING_X = -10, COLLECTING_Y = 45, COLLECTING_ANGLE = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        //voltageSensor = hardwareMap.voltageSensor.iterator().next();
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_ANGLE));
        outtakeSubsystem.init();
        intakeSubsystem.initAuto();
        outtakeSubsystem.claw.close();


        goToPreload = new Path(new BezierLine(new Point(START_X,START_Y, Point.CARTESIAN), new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)));
        goToPreload.setConstantHeadingInterpolation(Math.toRadians(START_ANGLE));
        follower.setMaxPower(preloadSpeed);

        goToCollectingPose = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE1_X, SAFE1_Y, Point.CARTESIAN),
                                new Point(COLLECTING_X, COLLECTING_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE), Math.toRadians(COLLECTING_ANGLE))
                .build();

        goToScore1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SCORE1_X, SCORE1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.setPathEndTimeoutConstraint(500)
                .build();

        goToScore2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SCORE2_X, SCORE2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.setPathEndTimeoutConstraint(500)
                .build();

        goToScore3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SCORE3_X, SCORE3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.setPathEndTimeoutConstraint(500)
                .build();

        goToScore4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SCORE4_X, SCORE4_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //.setPathEndTimeoutConstraint(500)
                .build();

        goToPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(PARK_X, PARK_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SPECIMEN_ANGLE), Math.toRadians(PARK_ANGLE))
                //.setPathEndTimeoutConstraint(500)
                .build();

        goToSample1 = follower.pathBuilder()
                .setLinearHeadingInterpolation(Math.toRadians(COLLECTING_ANGLE), Math.toRadians(SAMPLE1_HEADING))
                        .build();

        goToSample2 = follower.pathBuilder()
                .setLinearHeadingInterpolation(Math.toRadians(DROP_ANGLE), Math.toRadians(SAMPLE1_HEADING))
                .build();

        goToSample3 = follower.pathBuilder()
                .setLinearHeadingInterpolation(Math.toRadians(DROP_ANGLE), Math.toRadians(SAMPLE1_HEADING))
                .build();

        goToDrop = follower.pathBuilder()
                .setLinearHeadingInterpolation(Math.toRadians(follower.getTotalHeading()), Math.toRadians(DROP_ANGLE))
                .build();



        follower.followPath(goToPreload);

        waitForStart();

        while (opModeIsActive()) {
            //voltage = voltageSensor.getVoltage();

            switch(CS) {

            }


            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.addData("Scoring state", SCORING_CS);
            telemetry.addData("TransferState", robotSystems.transferState);
            telemetry.addData("TransferTimer", robotSystems.timer.milliseconds());
            telemetry.update();

            robotSystems.update();
            follower.update();
        }
    }
}
