package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.PositionsLift;
import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;
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
@Autonomous(name = "[AUTO] 5 Specimene V2", group = "HTECH")
public class Specimene5autoV2 extends LinearOpMode {

    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime timer2;
    ElapsedTime matchTimer;

    private Follower follower;
    private Path goToPreload;
    private PathChain goToCollectingPose;
    private PathChain goToWall, goToWall2;
    private PathChain goToScore1;
    private PathChain goToScore2;
    private PathChain goToScore3;
    private PathChain goToScore4;
    private PathChain goToPark;
    private PathChain goToSample1;
    private PathChain goToSample2;
    private PathChain goToSample3;
    private PathChain goToDrop;
    private PathChain goToHuman1, goToHuman2, goToHuman3;

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
        WAIT_TRANSFER,
        COLLECTING_SAMPLE,
        DROPPING_SAMPLE

    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;
    public int TIME_TO_WAIT2 = 0;

    public static double maxSpeed = 0.9;
    public static double slowSpeed = 0.65;
    public static double preloadSpeed = 0.6;

    public static double rotationSample1 = 0;
    public static double rotationSample3 = 0;
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

    public enum COLLECTING_STATES{
        IDLE,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        FINISHED
    }



    public boolean dropping = false;

    public COLLECTING_STATES COLLECTING_CS = COLLECTING_STATES.IDLE;

    public SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;

    public static int timeToPreload = 0;
    public static int timeToSample = 200;
    public static int timeToCollect = 100;
    public static int timeToCollect2 = 1000;
    public static int timeToCollect3 = 1000;
    public static int time_to_transfer = 2000;
    public static int time_to_lift = 650;
    public static int time_to_drop = 230;
    public static int time_to_close_extendo = 150;
    public static int time_to_open_extendo = 150;
    public static int time_to_close_claw = 100;
    public static int time_to_open_claw = 100;

    //start pose 135, 83

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -28, PRELOAD_Y = 0, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = -7, SAFE1_Y = -5;
    public static double SAMPLE1_X = -10, SAMPLE1_Y = 35, SAMPLE1_ANGLE = 200;
    public static double HUMAN1_X = -5, HUMAN1_Y = SAMPLE1_Y, HUMAN1_ANGLE = 180;

    public static double SAMPLE2_X = -10, SAMPLE2_Y = 36, SAMPLE2_ANGLE = 180;
    public static double HUMAN2_X = -5, HUMAN2_Y = SAMPLE2_Y, HUMAN2_ANGLE = 180;

    public static double SAMPLE3_X = -10, SAMPLE3_Y = 37, SAMPLE3_ANGLE = 160;
    public static double HUMAN3_X = -5, HUMAN3_Y = SAMPLE3_Y, HUMAN3_ANGLE = 180;

    public static double SAFE_WALL_X = -30, SAFE_WALL_Y = 30;

    public static double SCORE1_X = -28.5, SCORE1_Y = -6.5;
    public static double SCORE2_X = -28.5, SCORE2_Y = -8;
    public static double SCORE3_X = -28.5, SCORE3_Y = -9;
    public static double SCORE4_X = -28.5, SCORE4_Y = -10.5;

    public static double SPECIMEN_X = -9.5, SPECIMEN_Y = 45, SPECIMEN_ANGLE = 0; //pozitie specimen pe perete
    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 5;

    public static double PARK_X = -10, PARK_Y = 30, PARK_ANGLE = 80;


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

        goToSample1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE1_X, SAFE1_Y, Point.CARTESIAN),
                                new Point(SAMPLE1_X, SAMPLE1_Y,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE), Math.toRadians(SAMPLE1_ANGLE))
                .build();

        goToSample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                                new Point(SAMPLE2_X, SAMPLE2_Y,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE), Math.toRadians(SAMPLE2_ANGLE))
                .build();

        goToSample3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(SAMPLE3_X, SAMPLE3_Y,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE), Math.toRadians(SAMPLE3_ANGLE))
                .build();

//        goToHuman1 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
//                                new Point(HUMAN1_X, HUMAN1_Y, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(HUMAN1_ANGLE)
//                .build();
//
//        goToHuman2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
//                                new Point(HUMAN2_X, HUMAN2_Y, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(HUMAN2_ANGLE)
//                .build();
//
//        goToHuman3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
//                                new Point(HUMAN3_X, HUMAN3_Y, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(HUMAN3_ANGLE)
//                .build();

        goToWall = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(SAFE_WALL_X, SAFE_WALL_Y, Point.CARTESIAN),
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(HUMAN3_ANGLE), Math.toRadians(SPECIMEN_ANGLE))
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
                .build();



        follower.followPath(goToPreload);

        waitForStart();

        while (opModeIsActive()) {


            switch(CS) {
                case PRELOAD:

                    if (robotSystems.transferState == RobotSystems.TransferStates.IDLE) {
                        lift.goToHighChamber();
                        outtakeSubsystem.goToSpecimenScore();
                    }

                    if (lift.isAtPosition() && lift.target_position == PositionsLift.highChamber) {
                        timer.reset();
                        NS = STATES.PLACING_PRELOAD;
                        CS = STATES.MOVING;
                    }

                    break;

                case MOVING:
                    timer.reset();
                    if (NS == STATES.COLLECTING) {
                        if(follower.getCurrentTValue() > 0.6) {
                            follower.setMaxPower(slowSpeed);
                        }
                        if (follower.getCurrentTValue() > 0.97) {
                            follower.setMaxPower(0.3);
                            firstTime = true;
                            CS = NS;
                        }
                    }
                    if(!follower.isBusy() || follower.holdingPosition) {
                        firstTime = true;
                        CS = NS;
                    }
                    break;


                case WAITING:
                    if(timer.milliseconds() > TIME_TO_WAIT) {
                        if(dropping){
                            outtakeSubsystem.claw.open();
                            dropping = false;
                        }
                        CS = NS;
                    }
                    break;

                case PLACING_PRELOAD:
                    if(firstTime) {
                        lift.goToMagicPos();
                        firstTime = false;
                    }

                    if(lift.isAtPosition()) {
                        outtakeSubsystem.claw.open();
                        NS = STATES.SAMPLE1;
                        firstTime = true;
                        CS = STATES.WAITING;
                        TIME_TO_WAIT = timeToPreload;
                        timer.reset();
                    }

                    if(COLLECTING_CS == COLLECTING_STATES.IDLE){
                        COLLECTING_CS = COLLECTING_STATES.SAMPLE1;
                    }

                    if(SCORING_CS == SCORING_STATES.SPECIMEN1 && NS == STATES.SAMPLE1){
                        NS = STATES.BACK_TO_WALL;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN2 && NS == STATES.SAMPLE1){
                        NS = STATES.BACK_TO_WALL;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN3 && NS == STATES.SAMPLE1){
                        NS = STATES.BACK_TO_WALL;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN4 && NS == STATES.SAMPLE1){
                        firstTime = true;
                        timer.reset();
                        NS = STATES.PARK;
                    }
                    break;

                case COLLECTING_SAMPLE:
                    extendo.goToMax();

                    if(extendo.isAtPosition()) {
                        intakeSubsystem.collect(true);
                        outtakeSubsystem.claw.open();
                        timer.reset();
                        TIME_TO_WAIT = time_to_transfer;
                        CS = STATES.WAITING;
                        dropping = true;


                        if(COLLECTING_CS == COLLECTING_STATES.SAMPLE1){
                            COLLECTING_CS = COLLECTING_STATES.SAMPLE2;
                            NS = STATES.SAMPLE2;
                        }
                        else if(COLLECTING_CS == COLLECTING_STATES.SAMPLE2){
                            COLLECTING_CS = COLLECTING_STATES.SAMPLE3;
                            NS = STATES.SAMPLE3;
                        }
                        else if(COLLECTING_CS == COLLECTING_STATES.SAMPLE3){
                            COLLECTING_CS = COLLECTING_STATES.FINISHED;
                            NS = STATES.WALL;
                        }
                        break;
                    }


                case SAMPLE1:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample1, true);
                    lift.goToGround();
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SAMPLE;
                    break;

                case SAMPLE2:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample2, true);
                    outtakeSubsystem.joint.goToBasketScore();
                    //extendo.goToMax();
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SAMPLE;
                    break;

                case SAMPLE3:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample3, true);
                    outtakeSubsystem.joint.goToBasketScore();
                    //extendo.goToMax();
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SAMPLE;
                    break;

                case WALL:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToWall, true);
                    intakeSubsystem.goToWall();
                    if(SCORING_CS == SCORING_STATES.IDLE){
                        SCORING_CS = SCORING_STATES.SPECIMEN1;
                    }
                    NS = STATES.COLLECTING;
                    CS = STATES.MOVING;
                    break;

                case COLLECTING:
                    intakeSubsystem.claw.close();
                    TIME_TO_WAIT = timeToCollect;
                    timer.reset();
                    CS = STATES.WAITING;
                    NS = STATES.TRANSFERING;
                    break;

                case TRANSFERING:
                    robotSystems.startTransfer();
                    TIME_TO_WAIT = time_to_transfer;
                    timer.reset();
                    CS = STATES.WAITING;
                    NS = STATES.SCORE;
                    break;

                case SCORE:
                    follower.setMaxPower(maxSpeed);
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1) {
                        follower.followPath(goToScore1, true);
                        CS = STATES.PRELOAD;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        follower.followPath(goToScore2, true);
                        CS = STATES.PRELOAD;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        follower.followPath(goToScore3, true);
                        CS = STATES.PRELOAD;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN4){
                        follower.followPath(goToScore4, true);
                        CS = STATES.PRELOAD;
                    }
                    break;
                case BACK_TO_WALL:
                    intakeSubsystem.goToWall();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToWall2, true);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING;
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        SCORING_CS = SCORING_STATES.SPECIMEN2;
                    }else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        SCORING_CS = SCORING_STATES.SPECIMEN3;
                    }else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        SCORING_CS = SCORING_STATES.SPECIMEN4;
                    }
                    break;
                case PARK:
                    follower.setMaxPower(maxSpeed);
                    lift.goToGround();
                    extendo.goToGround();
                    follower.followPath(goToPark);
                    CS = STATES.MOVING;
                    NS = STATES.PARKED;
                    break;
                case PARKED:
                    //end
                    break;
            }


            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.addData("Scoring state", SCORING_CS);
            telemetry.addData("Collecting state", COLLECTING_CS);
            telemetry.addData("TransferState", robotSystems.transferState);
            telemetry.addData("TransferTimer", robotSystems.timer.milliseconds());
            telemetry.update();

            robotSystems.update();
            follower.update();
        }
    }
}

