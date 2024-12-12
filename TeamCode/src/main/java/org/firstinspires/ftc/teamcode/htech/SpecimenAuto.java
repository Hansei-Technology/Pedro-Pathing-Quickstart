package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class SpecimenAuto extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;

    private Follower follower;
    private Path safeScoring;
    private Path goToPreload;
    private PathChain goTo1Sample;
    private PathChain goTo2Sample;
    private PathChain goTo3Sample;
    private PathChain goToWall;
    private PathChain goToScore;
    private PathChain goToWall2;
    private PathChain goToPark;
    private PathChain goToSafe4;

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
        COLLECTING1,
        COLLECTING2,
        COLLECTING3,
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3,
        PARK,
        PARKED,
        SAFE4,
        SCORE1,
        SCORE2,
        SCORE3,
        SAFE_SCORE
    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;

    boolean firstTime = true;

    public enum SCORING_STATES{
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3,
        IDLE
    }

    public SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;

    public static int timeToPreload = 400;
    public static int timeToSample = 200;
    public static int timeToCollect1 = 1000;
    public static int timeToCollect2 = 1000;
    public static int timeToCollect3 = 1000;
    public static int time_to_transfer = 600;
    public static int time_to_lift = 650;
    public static int time_to_drop = 800;

    //start pose 135, 83

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = 108.5 -135, PRELOAD_Y = -5, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = -20, SAFE1_Y = 10, SAFE11_ANGLE;
    public static double SAMPLE1_X = -48, SAMPLE1_Y = 35, SAMPLE1_ANGLE = 0;
    public static double HUMAN1_X = -15, HUMAN1_Y = SAMPLE1_Y, HUMAN1_ANGLE = 0;

    public static double SAFE2_X = -48, SAFE2_Y = 13, SAFE2_ANGLE;
    public static double SAMPLE2_X = -48, SAMPLE2_Y = 45, SAMPLE2_ANGLE = 0;
    public static double HUMAN2_X = -15, HUMAN2_Y = SAMPLE2_Y, HUMAN2_ANGLE = 0;

    public static double SAFE3_X = -40, SAFE3_Y = 33, SAFE3_ANGLE;
    public static double SAMPLE3_X = -48, SAMPLE3_Y = 52, SAMPLE3_ANGLE = 0;
    public static double HUMAN3_X = -15, HUMAN3_Y = SAMPLE3_Y, HUMAN3_ANGLE = 0;
    public static double SAFE4_X = -20, SAFE4_Y = 30, SAFE4_ANGLE = 0;

    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 0, SAFE_SPECIMEN_ANGLE;

    public static double SPECIMEN1_X = -9, SPECIMEN1_Y = 30, SPECIMEN1_ANGLE = 0;
    public static double PARK_X = -15, PARK_Y = 34, PARK_ANGLE = 90;
    public static double SAFE_PARK_X = -52, SAFE_PARK_Y = -38, SAFE_PARK_ANGLE;

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

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_ANGLE));
        outtakeSubsystem.init();
        intakeSubsystem.initAuto();
        outtakeSubsystem.claw.close();


        goToPreload = new Path(new BezierLine(new Point(START_X,START_Y, Point.CARTESIAN), new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)));
        goToPreload.setConstantHeadingInterpolation(Math.toRadians(START_ANGLE));
//        goToPreload.setReversed(true);
        follower.setMaxPower(0.6);
        safeScoring = new Path(new BezierLine(new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN), new Point(SAFE4_X, SAFE4_Y, Point.CARTESIAN)));
        safeScoring.setConstantHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE));

        goTo1Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE1_X, SAFE1_Y, Point.CARTESIAN),
                                //new Point(SAFE12_X, SAFE12_Y, Point.CARTESIAN),
                                new Point(SAMPLE1_X,SAMPLE1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE))
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                                new Point(HUMAN1_X, HUMAN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(HUMAN1_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goTo2Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(HUMAN1_X,HUMAN1_Y, Point.CARTESIAN),
                                new Point(SAFE2_X, SAFE2_Y, Point.CARTESIAN),
                                new Point(SAMPLE2_X,SAMPLE2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE))
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(HUMAN2_X, HUMAN2_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(HUMAN2_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goTo3Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(HUMAN2_X,HUMAN2_Y, Point.CARTESIAN),
                                new Point(SAFE3_X, SAFE3_Y, Point.CARTESIAN),
                                new Point(SAMPLE3_X,SAMPLE3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE))
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(HUMAN3_X, HUMAN3_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(HUMAN3_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToSafe4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(HUMAN3_X, HUMAN3_Y, Point.CARTESIAN),
                                new Point(SAFE4_X, SAFE4_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SAFE4_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToWall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAFE4_X, SAFE4_Y, Point.CARTESIAN),
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(Math.toRadians(PRELOAD_ANGLE)))
                .setPathEndTimeoutConstraint(500)
                .build();


        goToPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(PARK_X, PARK_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SPECIMEN1_ANGLE), Math.toRadians(PARK_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToScore = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SPECIMEN1_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToWall2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SPECIMEN1_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        follower.followPath(goToPreload);

        waitForStart();

        while (opModeIsActive()) {

            switch(CS) {
                case PRELOAD:
                    lift.goToHighChamber();
                    outtakeSubsystem.goToSpecimenScore();
                    NS = STATES.PLACING_PRELOAD;
                    CS = STATES.MOVING;
                    break;

                case MOVING:
                    if(!follower.isBusy() || follower.holdingPosition/* ||  follower.getCurrentTValue() > 0.99 */) {
                        firstTime = true;
                        CS = NS;
                    }
                    break;

                case WAITING:
                    if(timer.milliseconds() > TIME_TO_WAIT) {
                            CS = NS;
                    }
                    break;

                case TRANSFERING:
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE) {
                        if(firstTime) {
                            timer.reset();
                            firstTime = false;
                        }
                        if(timer.milliseconds() > time_to_transfer) {
//                            lift.goToHighChamber();
                            firstTime = true;
                            CS = STATES.MOVING;
                        }
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

                    if(SCORING_CS == SCORING_STATES.SPECIMEN1 && NS == STATES.SAMPLE1){
                        NS = STATES.SAFE_SCORE;
                        break;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN2 && NS == STATES.SAMPLE1){
                        NS = STATES.SAFE_SCORE;
                        break;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN3 && NS == STATES.SAMPLE1){
                        NS = STATES.PARK;
                        break;
                    }
                    break;
                case SAMPLE1:
                    follower.setMaxPower(0.85);
                    follower.followPath(goTo1Sample);
                    outtakeSubsystem.goToTransfer();
                    lift.goToGround();
                    intakeSubsystem.goToWall();
                    intakeSubsystem.claw.open();

                    timer.reset();
                    NS = STATES.SAMPLE2;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAMPLE2:
                    follower.setMaxPower(0.85);
                    follower.followPath(goTo2Sample);

                    NS = STATES.SAMPLE3;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAMPLE3:
                    follower.setMaxPower(0.85);
                    follower.followPath(goTo3Sample);

                    NS = STATES.SAFE4;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAFE4:
                    follower.setMaxPower(0.6);
                    follower.followPath(goToSafe4, true);
                    NS = STATES.COLLECTING1;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case COLLECTING1:
                    follower.setMaxPower(0.5);
                    follower.followPath(goToWall);
                    if(SCORING_CS == SCORING_STATES.IDLE){
                        SCORING_CS = SCORING_STATES.SPECIMEN1;
                    }
                    NS = STATES.COLLECTING2;
                    CS = STATES.MOVING;
                    break;
                case COLLECTING2:
                    intakeSubsystem.claw.close();
                    robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN;
                    CS = STATES.TRANSFERING;
                    NS = STATES.SCORE1;
                    break;
                case SCORE1:
                    follower.setMaxPower(0.6);
                    follower.followPath(goToScore);
                    CS = STATES.PRELOAD;
                    break;
                case SAFE_SCORE:
                    intakeSubsystem.goToWall();
                    robotSystems.transferState = RobotSystems.TransferStates.INTAKE_WALL;
                    follower.setMaxPower(0.75);
                    follower.followPath(goToWall2, true);
                    CS = STATES.COLLECTING1;
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        SCORING_CS = SCORING_STATES.SPECIMEN2;
                    }else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        SCORING_CS = SCORING_STATES.SPECIMEN3;
                    }
                    break;
                case PARK:
                    follower.setMaxPower(0.8);
                    follower.followPath(goToPark);
                    lift.goToGround();
            }


            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.update();

            robotSystems.update();
            follower.update();
        }
    }
}
