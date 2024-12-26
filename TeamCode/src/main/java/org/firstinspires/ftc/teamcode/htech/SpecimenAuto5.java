package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.PositionsLift;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name = "[AUTO] 5 Specimene", group = "HTECH")
public class SpecimenAuto5 extends LinearOpMode {

    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;

    ElapsedTime timer;
    Follower follower;

    PathChain goToPreload;
    PathChain goToSample1, goToCollect1;
    PathChain goToSample2, goToCollect2;
    PathChain goToSample3, goToCollect3;
    PathChain goToWall, backToWall;
    PathChain goToScore1, goToScore2, goToScore3, goToScore4;
    PathChain goToPark;


    //STATES
    private enum STATES{
        NULL,
        MOVING, WAITING, TRANSFERING,
        SPECIMEN, PLACING_SPECIMEN,
        SAMPLE1, SAMPLE2, SAMPLE3,
        COLLECTING1, COLLECTING2, COLLECTING3, COLLECTING_SPECIMEN,
        WALL, BACK_TO_WALL,
        SCORE,
        PARK,
        PARKED
    }
    public enum SCORING_STATES{
        IDLE,
        SPECIMEN1, SPECIMEN2, SPECIMEN3, SPECIMEN4
    }
    STATES CS = STATES.NULL, NS = STATES.NULL;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;


    //Timers
    public static int timeToCollect = 100;
    public static int timeToTransfer = 800;
    public int TIME_TO_WAIT = 0;


    //Speeds
    public static double maxSpeed = 1;
    public static double mediumSpeed = 0.8;
    public static double slowSpeed = 0.6;


    //Booleans
    public boolean dropping = false;
    public boolean sample3 = false;


    //POSES
    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -28.5, PRELOAD_Y = 0, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE_X = 0, SAFE_Y = 12;
    public static double SAMPLE1_X = -20, SAMPLE1_Y = 37, SAMPLE1_ANGLE = 180;

    public static double SAMPLE2_X = -20, SAMPLE2_Y = 45, SAMPLE2_ANGLE = 180;
    public static double SAMPLE3_X = -20, SAMPLE3_Y = 52.3, SAMPLE3_ANGLE = 180;

    public static double SCORE1_X = -28.5, SCORE1_Y = -3;
    public static double SCORE2_X = -28.5, SCORE2_Y = -4;
    public static double SCORE3_X = -28.5, SCORE3_Y = -5;
    public static double SCORE4_X = -28.5, SCORE4_Y = -6;

    public static double SAFE_WALL_X = -48, SAFE_WALL_Y = 40;
    public static double SPECIMEN_X = -8.3, SPECIMEN_Y = 30, SPECIMEN_ANGLE = 0;
    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 5;
    public static double SAFE_SPECIMEN2_X = -20, SAFE_SPECIMEN2_Y = SPECIMEN_Y;

    public static double PARK_X = -10, PARK_Y = 30, PARK_ANGLE = 80;

    public static double COLLECT1_X = SAMPLE1_X, COLLECT1_Y = SAMPLE1_Y, COLLECT1_ANGLE = SAMPLE1_ANGLE;
    public static double COLLECT2_X = SAMPLE2_X, COLLECT2_Y = SAMPLE2_Y, COLLECT2_ANGLE = SAMPLE2_ANGLE;
    public static double COLLECT3_X = SAMPLE3_X, COLLECT3_Y = SAMPLE3_Y, COLLECT3_ANGLE = SAMPLE3_ANGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_ANGLE));

        //init
        outtakeSubsystem.init();
        intakeSubsystem.initAuto();
        outtakeSubsystem.claw.close();

        //Trajectories
        goToPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(START_X,START_Y, Point.CARTESIAN),
                                new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE))
                .build();

        goToSample1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE_X, SAFE_Y, Point.CARTESIAN),
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE), Math.toRadians(SAMPLE1_ANGLE))
                .build();

        goToSample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE))
                .build();

        goToSample3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE))
                .build();

        goToCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(COLLECT1_X, COLLECT1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(COLLECT1_ANGLE))
                .build();

        goToCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(COLLECT2_X, COLLECT2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(COLLECT2_ANGLE))
                .build();

        goToCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(COLLECT3_X, COLLECT3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(COLLECT3_ANGLE))
                .build();

        goToWall = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(SAFE_WALL_X, SAFE_WALL_Y, Point.CARTESIAN),
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE), Math.toRadians(SPECIMEN_ANGLE))
                .build();

        backToWall = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN2_X, SAFE_SPECIMEN2_Y, Point.CARTESIAN),
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SPECIMEN_ANGLE))
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

        goToPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(PARK_X, PARK_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SPECIMEN_ANGLE), Math.toRadians(PARK_ANGLE))
                .build();

        //Starting trajectory
        follower.setMaxPower(maxSpeed);
        follower.followPath(goToPreload);
        CS = STATES.SPECIMEN;

        waitForStart();

        while(opModeIsActive()){
            
            switch(CS){

                case SPECIMEN:
                    lift.goToHighChamber();
                    outtakeSubsystem.goToSpecimenScore();
                    if (lift.isAtPosition() && lift.target_position == PositionsLift.highChamber) {
                        timer.reset();
                        NS = STATES.PLACING_SPECIMEN;
                        CS = STATES.MOVING;
                    }
                    break;

                case MOVING:
                    timer.reset();
                    CS = NS;
                    if(!follower.isBusy()) {
                        break;
                    }

                case WAITING:
                    if(timer.milliseconds() > TIME_TO_WAIT){
                        CS = NS;
                        if(dropping){
                            outtakeSubsystem.joint.goToBasketScore();
                            dropping = false;
                        }
                        if(sample3){
                            outtakeSubsystem.claw.open();
                            sample3 = false;
                        }
                    }
                    break;

                case PLACING_SPECIMEN:
                    robotSystems.scoreSpecimen();
                    if(SCORING_CS == SCORING_STATES.IDLE){
                        NS = STATES.SAMPLE1;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        NS = STATES.BACK_TO_WALL;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        NS = STATES.BACK_TO_WALL;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        NS = STATES.BACK_TO_WALL;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN4){
                        NS = STATES.PARK;
                    }
                    break;

                case SAMPLE1:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample1);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING1;
                    break;

                case COLLECTING1:
                    extendo.goToMax();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToCollect1, true);
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    if(extendo.isAtPosition()){
                        intakeSubsystem.collect(true);
                        CS = STATES.WAITING;
                        NS = STATES.SAMPLE2;
                        timer.reset();
                        TIME_TO_WAIT = timeToTransfer;
                        dropping = true;
                    }
                    break;

                case SAMPLE2:
                    outtakeSubsystem.claw.open();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample2);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING2;
                    break;

                case COLLECTING2:
                    extendo.goToMax();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToCollect2, true);
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    if(extendo.isAtPosition()){
                        intakeSubsystem.collect(true);
                        CS = STATES.WAITING;
                        NS = STATES.SAMPLE3;
                        timer.reset();
                        TIME_TO_WAIT = timeToTransfer;
                        dropping = true;
                    }
                    break;

                case SAMPLE3:
                    outtakeSubsystem.claw.open();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToSample3);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING2;
                    break;

                case COLLECTING3:
                    extendo.goToMax();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToCollect3, true);
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    if(extendo.isAtPosition()){
                        intakeSubsystem.collect(true);
                        CS = STATES.WAITING;
                        NS = STATES.WALL;
                        timer.reset();
                        TIME_TO_WAIT = timeToTransfer;
                        dropping = true;
                        sample3 = true;
                    }
                    break;

                case WALL:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToWall, true);
                    NS = STATES.COLLECTING_SPECIMEN;
                    CS = STATES.MOVING;
                    break;

                case COLLECTING_SPECIMEN:
                    intakeSubsystem.claw.close();
                    TIME_TO_WAIT = timeToCollect;
                    timer.reset();
                    CS = STATES.WAITING;
                    NS = STATES.TRANSFERING;
                    break;

                case TRANSFERING:
                    robotSystems.startTransfer();
                    TIME_TO_WAIT = timeToTransfer;
                    timer.reset();
                    CS = STATES.WAITING;
                    NS = STATES.SCORE;
                    break;

                case SCORE:
                    lift.goToHighChamber();
                    follower.setMaxPower(maxSpeed);
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1) {
                        follower.followPath(goToScore1, true);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        follower.followPath(goToScore2, true);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        follower.followPath(goToScore3, true);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN4){
                        follower.followPath(goToScore4, true);
                        CS = STATES.SPECIMEN;
                    }
                    break;

                case BACK_TO_WALL:
                    intakeSubsystem.goToWall();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(backToWall, true);
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        SCORING_CS = SCORING_STATES.SPECIMEN2;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        SCORING_CS = SCORING_STATES.SPECIMEN3;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
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
                    break;

            }

            //telemetries
            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.addData("TransferState", robotSystems.transferState);
            telemetry.addData("TransferTimer", robotSystems.timer.milliseconds());

            //updates
            telemetry.update();
            robotSystems.update();
            follower.update();
        }
    }
}
