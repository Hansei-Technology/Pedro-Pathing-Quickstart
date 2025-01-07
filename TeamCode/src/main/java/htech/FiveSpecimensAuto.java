package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.subsystem.ChassisMovement;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
@Autonomous(name = "[AUTO] 5+0", group = "HTECH")
public class FiveSpecimensAuto extends LinearOpMode {


    //Mechanisms
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;


    //Pedro
    Follower follower;
    Path goToPreload;
    PathChain goToCollectSamples;
    PathChain goToWall;
    PathChain goToScore1;
    PathChain goToScore2;
    PathChain goToScore3;
    PathChain goToScore4;
    PathChain goToPark;


    //States
    private enum STATES{
        IDLE,
        SPECIMEN, PLACING_SPECIMEN,
        MOVING, WAITING, TRANSFERING,
        COLLECTING_SAMPLES, COLLECTING_SPECIMEN,
        SCORE, WALL,
        PARK, PARKED
    }

    private enum SCORING_STATES{
        IDLE,
        SPECIMEN1, SPECIMEN2, SPECIMEN3, SPECIMEN4
    }

    public enum TRAJECTORY_STATES{
        IDLE,
        PRELOAD,
        COLLECTING_SAMPLES,
        GO_TO_SCORE1, GO_TO_SCORE2, GO_TO_SCORE3, GO_TO_SCORE4,
        GO_TO_WALL,
        GO_TO_PARK
    }

    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;
    TRAJECTORY_STATES TRAJECTORY_CS = TRAJECTORY_STATES.IDLE;


    //Timers
    public static int timeToScoreSpecimen = 200;
    public static int timeToTransfer = 650;
    public static int timeToCollectSpecimen = 75;
    public int TIME_TO_WAIT = 0;


    //Booleans
    public boolean collectingSpecimen = false;
    public boolean collectingSample = false;


    //Speeds
    public static double maxSpeed = 0.9;
    public static double scoreSpeed = 0.7;
    public static double sample3Speed = 0.7;
    public static double collectSpeed = 0.6;


    //Poses
    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -27, PRELOAD_Y = 0, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = -5, SAFE1_Y = 30;
    public static double SAFE12_X = -30, SAFE12_Y = 10;
    public static double SAFE13_X = -60, SAFE13_Y = 37;
    public static double SAMPLE1_X = -48, SAMPLE1_Y = 37, SAMPLE1_ANGLE = 0;
    public static double HUMAN1_X = -20, HUMAN1_Y = SAMPLE1_Y, HUMAN1_ANGLE = 0;

    public static double SAFE2_X = -48, SAFE2_Y = 30;
    public static double SAMPLE2_X = -48, SAMPLE2_Y = 45, SAMPLE2_ANGLE = 0;
    public static double HUMAN2_X = -20, HUMAN2_Y = SAMPLE2_Y, HUMAN2_ANGLE = 0;

    public static double SAFE3_X = -40, SAFE3_Y = 40;
    public static double SAMPLE3_X = -48, SAMPLE3_Y = 52.3, SAMPLE3_ANGLE = 0;
    public static double SPECIMEN1_X = -7.9, SPECIMEN1_Y = SAMPLE3_Y, SPECIMEN1_ANGLE = 0;

    public static double SCORE1_X = -27, SCORE1_Y = 1;
    public static double SCORE2_X = -27, SCORE2_Y = -1;
    public static double SCORE3_X = -27, SCORE3_Y = -2.5;
    public static double SCORE4_X = -27, SCORE4_Y = -3.5;
    public static double SAFE_SCORE_X = -14, SAFE_SCORE_Y = 0;

    public static double SPECIMEN_X = -8.4, SPECIMEN_Y = 30, SPECIMEN_ANGLE = 0;

    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 5;
    public static double SAFE_SPECIMEN2_X = -20, SAFE_SPECIMEN2_Y = SPECIMEN_Y;

    public static double PARK_X = -10, PARK_Y = 30, PARK_ANGLE = 80;

    public double TARGET_X = 0, TARGET_Y = 0;


    //Methods
    public boolean finishedPath(){
        return !follower.isBusy() || ((follower.getPose().getX() < (TARGET_X + 1) && follower.getPose().getX() > (TARGET_X - 1)) && (follower.getPose().getY() < (TARGET_Y + 1) && follower.getPose().getY() > (TARGET_Y - 1))) || follower.getCurrentTValue() > 0.99;
    }

    public boolean slowForSample3(){
        return (follower.getPose().getX() < (SAMPLE3_X + 1) && follower.getPose().getX() > (SAMPLE3_X - 1)) && (follower.getPose().getY() < (SAMPLE3_Y + 1) && follower.getPose().getY() > (SAMPLE3_Y - 1));
    }

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
        matchTimer = new ElapsedTime();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_ANGLE));


        //Init
        outtakeSubsystem.init();
        intakeSubsystem.initAuto();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;


        //Paths
        goToPreload = new Path(
                new BezierLine(
                        new Point(START_X,START_Y, Point.CARTESIAN),
                        new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)
                ));
        goToPreload.setConstantHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE));

        goToCollectSamples = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE1_X, SAFE1_Y, Point.CARTESIAN),
                                new Point(SAFE12_X, SAFE12_Y, Point.CARTESIAN),
                                new Point(SAFE13_X, SAFE13_Y, Point.CARTESIAN),
                                new Point(SAMPLE1_X,SAMPLE1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                                new Point(HUMAN1_X, HUMAN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(HUMAN1_ANGLE))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(HUMAN1_X,HUMAN1_Y, Point.CARTESIAN),
                                new Point(SAFE2_X, SAFE2_Y, Point.CARTESIAN),
                                new Point(SAMPLE2_X,SAMPLE2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(HUMAN2_X, HUMAN2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(HUMAN2_ANGLE))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(HUMAN2_X,HUMAN2_Y, Point.CARTESIAN),
                                new Point(SAFE3_X, SAFE3_Y, Point.CARTESIAN),
                                new Point(SAMPLE3_X,SAMPLE3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SPECIMEN1_ANGLE))
                .build();

        goToScore1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN),
                                new Point(SAFE_SCORE_X, SAFE_SCORE_Y, Point.CARTESIAN),
                                new Point(SCORE1_X, SCORE1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SCORE_X, SAFE_SCORE_Y, Point.CARTESIAN),
                                new Point(SCORE2_X, SCORE2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SCORE_X, SAFE_SCORE_Y, Point.CARTESIAN),
                                new Point(SCORE3_X, SCORE3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToScore4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SAFE_SCORE_X, SAFE_SCORE_Y, Point.CARTESIAN),
                                new Point(SCORE4_X, SCORE4_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToWall = follower.pathBuilder()
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
        TRAJECTORY_CS = TRAJECTORY_STATES.PRELOAD;

        waitForStart();

        //Match timer
        matchTimer.reset();

        while(opModeIsActive() && matchTimer.seconds() < 30.5){

            //State machine for autonomous
            switch(CS){

                case SPECIMEN:
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                        follower.setMaxPower(maxSpeed);
                        lift.goToHighChamber();
                        outtakeSubsystem.goToSpecimenScore();
                        timer.reset();
                        NS = STATES.PLACING_SPECIMEN;
                        CS = STATES.MOVING;
                    }
                    break;

                case WAITING:
                    if(timer.milliseconds() > TIME_TO_WAIT){
                        CS = NS;
                    }
                    break;

                case MOVING:
                    //State machine for trajectory
//                    switch(TRAJECTORY_CS){
//
//                        case IDLE:
//                            break;
//
//                        case PRELOAD:
//                            TARGET_X = PRELOAD_X;
//                            TARGET_Y = PRELOAD_Y;
//                            break;
//
//                        case COLLECTING_SAMPLES:
//                            TARGET_X = SPECIMEN1_X;
//                            TARGET_Y = SPECIMEN1_Y;
//                            break;
//
//                        case GO_TO_SCORE1:
//                            TARGET_X = SCORE1_X;
//                            TARGET_Y = SCORE1_Y;
//                            break;
//
//                        case GO_TO_SCORE2:
//                            TARGET_X = SCORE2_X;
//                            TARGET_Y = SCORE2_Y;
//                            break;
//
//                        case GO_TO_SCORE3:
//                            TARGET_X = SCORE3_X;
//                            TARGET_Y = SCORE3_Y;
//                            break;
//
//                        case GO_TO_SCORE4:
//                            TARGET_X = SCORE4_X;
//                            TARGET_Y = SCORE4_Y;
//                            break;
//
//                        case GO_TO_WALL:
//                            TARGET_X = SPECIMEN_X;
//                            TARGET_Y = SPECIMEN_Y;
//                            break;
//
//                        case GO_TO_PARK:
//                            TARGET_X = PARK_X;
//                            TARGET_Y = PARK_Y;
//                            break;
//
//                    }
                    timer.reset();
//                    if(collectingSample){
//                        if(slowForSample3()){
//                            follower.setMaxPower(sample3Speed);
//                            collectingSample = false;
//                        }
//                    }
//                    if(collectingSpecimen){
//                        if(follower.getCurrentTValue() >= 0.6){
//                            follower.setMaxPower(collectSpeed);
//                            collectingSpecimen = false;
//                        }
//                    }
                    if(!follower.isBusy()){
                        CS = NS;
                    }
                    break;

                case PLACING_SPECIMEN:
                    lift.goToMagicPos();
                    if(timer.milliseconds() > timeToScoreSpecimen){
                        outtakeSubsystem.claw.open();
                        if(SCORING_CS == SCORING_STATES.IDLE){
                            CS = STATES.COLLECTING_SAMPLES;
                        }
                        if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                            CS = STATES.WALL;
                        }
                        if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                            CS = STATES.WALL;
                        }
                        if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                            CS = STATES.WALL;
                        }
                        if(SCORING_CS == SCORING_STATES.SPECIMEN4){
                            CS = STATES.PARK;
                        }
                    }
                    break;

                case COLLECTING_SAMPLES:
                    collectingSample = true;
                    lift.goToGround();
                    extendo.goToGround();
                    intakeSubsystem.goToWall();
                    intakeSubsystem.claw.open();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToCollectSamples);
                    TRAJECTORY_CS = TRAJECTORY_STATES.COLLECTING_SAMPLES;
                    lift.goToGround();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    SCORING_CS = SCORING_STATES.SPECIMEN1;
                    break;

                case COLLECTING_SPECIMEN:
                    intakeSubsystem.claw.close();
                    TIME_TO_WAIT = timeToCollectSpecimen;
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
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1) {
                        follower.followPath(goToScore1);
                        TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_SCORE1;
                        follower.setMaxPower(maxSpeed);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        follower.followPath(goToScore2);
                        TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_SCORE2;
                        follower.setMaxPower(scoreSpeed);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        follower.followPath(goToScore3);
                        TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_SCORE3;
                        follower.setMaxPower(scoreSpeed);
                        CS = STATES.SPECIMEN;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN4) {
                        follower.followPath(goToScore4);
                        TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_SCORE4;
                        follower.setMaxPower(scoreSpeed);
                        CS = STATES.SPECIMEN;
                    }
                    break;

                case WALL:
                    collectingSpecimen = true;
                    lift.goToGround();
                    intakeSubsystem.goToWall();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToWall);
                    TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_WALL;
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
                    TRAJECTORY_CS = TRAJECTORY_STATES.GO_TO_PARK;
                    CS = STATES.MOVING;
                    NS = STATES.PARKED;
                    break;

                case PARKED:
                    //end
                    break;

            }




            //Telemetry
            telemetry.addData("State", CS);
            telemetry.addData("Next State", NS);
            telemetry.addData("Scoring State", SCORING_CS);
            telemetry.addData("Trajectory State", TRAJECTORY_CS);
            telemetry.addData("Match timer", matchTimer.seconds());


            //Updates
            robotSystems.update();
            follower.update();
            telemetry.update();
        }
    }


}
