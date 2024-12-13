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
    private PathChain goToScore2;
    private PathChain goToScore3;
    private PathChain goTo1Sample;
    private PathChain goTo2Sample;
    private PathChain goTo3Sample;
    private PathChain goToWall;
    private PathChain goToScore1;
    private PathChain goToWall2;
    private PathChain goToPark;
    private PathChain goToCheckpoint;

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
        CHECKPOINT

    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;

    public static double maxSpeed = 1;
    public static double slowSpeed = 0.5;
    public static double preloadSpeed = 0.6;

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
    public static int time_to_grab = 100;

    //start pose 135, 83

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = 108.5 -135, PRELOAD_Y = -5, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = -20, SAFE1_Y = 10;
    public static double SAMPLE1_X = -48, SAMPLE1_Y = 35, SAMPLE1_ANGLE = 0;
    public static double HUMAN1_X = -15, HUMAN1_Y = SAMPLE1_Y, HUMAN1_ANGLE = 0;

    public static double SAFE2_X = -48, SAFE2_Y = 13;
    public static double SAMPLE2_X = -48, SAMPLE2_Y = 45, SAMPLE2_ANGLE = 0;
    public static double HUMAN2_X = -15, HUMAN2_Y = SAMPLE2_Y, HUMAN2_ANGLE = 0;

    public static double SAFE3_X = -40, SAFE3_Y = 33;
    public static double SAMPLE3_X = -48, SAMPLE3_Y = 52, SAMPLE3_ANGLE = 0;
    public static double HUMAN3_X = -15, HUMAN3_Y = SAMPLE3_Y, HUMAN3_ANGLE = 0;
    public static double CHECKPOINT_X = -20, CHECKPOINT_Y = 30, CHECKPOINT_ANGLE = 0;

    public static double SCORE1_X = 108.5 -135, SCORE1_Y = -5;
    public static double SCORE2_X = 108.5 -135, SCORE2_Y = -5;
    public static double SCORE3_X = 108.5 -135, SCORE3_Y = -5;

    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = 0;

    public static double SPECIMEN_X = -9, SPECIMEN_Y = 30, SPECIMEN_ANGLE = 0;
    public static double PARK_X = -15, PARK_Y = 34, PARK_ANGLE = 90;

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
        follower.setMaxPower(preloadSpeed);
        safeScoring = new Path(new BezierLine(new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN), new Point(CHECKPOINT_X, CHECKPOINT_Y, Point.CARTESIAN)));
        safeScoring.setConstantHeadingInterpolation(Math.toRadians(PRELOAD_ANGLE));

        goTo1Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE1_X, SAFE1_Y, Point.CARTESIAN),
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

        goToCheckpoint = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(HUMAN3_X, HUMAN3_Y, Point.CARTESIAN),
                                new Point(CHECKPOINT_X, CHECKPOINT_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(CHECKPOINT_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToWall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(CHECKPOINT_X, CHECKPOINT_Y, Point.CARTESIAN),
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN)
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
                .setLinearHeadingInterpolation(Math.toRadians(SPECIMEN_ANGLE), Math.toRadians(PARK_ANGLE))
                .setPathEndTimeoutConstraint(500)
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
                .setPathEndTimeoutConstraint(500)
                .build();

        goToWall2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE_SPECIMEN_X, SAFE_SPECIMEN_Y, Point.CARTESIAN),
                                new Point(SPECIMEN_X, SPECIMEN_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(SPECIMEN_ANGLE))
                .setPathEndTimeoutConstraint(500)
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
                .setPathEndTimeoutConstraint(500)
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
                        NS = STATES.BACK_TO_WALL;
                        break;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN2 && NS == STATES.SAMPLE1){
                        NS = STATES.BACK_TO_WALL;
                        break;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN3 && NS == STATES.SAMPLE1){
                        NS = STATES.PARK;
                        break;
                    }
                    break;
                case SAMPLE1: //aduce sample 1 la human
                    follower.setMaxPower(maxSpeed);
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
                case SAMPLE2: //aduce sample 2 la human
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goTo2Sample);

                    NS = STATES.SAMPLE3;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAMPLE3: //aduce sample 3 la human
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goTo3Sample);

                    NS = STATES.CHECKPOINT;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case CHECKPOINT: //fostul SAFE4; merge la checkpoint(punctul inainte de perete)
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToCheckpoint, true);
                    NS = STATES.WALL;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case WALL: //fostul COLLECTING1; merge la perete pentru a lua specimenul
                    follower.setMaxPower(slowSpeed);
                    follower.followPath(goToWall);
                    if(SCORING_CS == SCORING_STATES.IDLE){
                        SCORING_CS = SCORING_STATES.SPECIMEN1;
                    }
                    NS = STATES.COLLECTING;
                    CS = STATES.MOVING;
                    break;
                case COLLECTING: //fostul COLLECTING2; prinde specimenul
                    intakeSubsystem.claw.close();
                    CS = STATES.WAITING;
                    NS = STATES.SCORE;
                    TIME_TO_WAIT = time_to_grab; //wait-ul de 100 ms ca sa aiba timp clestele sa prinda specimenul
                    break;
                case SCORE: //merge sa puncteze specimenul si face transfer
                    follower.setMaxPower(maxSpeed);
                    robotSystems.startTransfer();
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1) {  //goToScore este diferit ca sa nu puna specimenul unul peste altul
                        follower.followPath(goToScore1);
                        CS = STATES.PRELOAD;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        follower.followPath(goToScore2);
                        CS = STATES.PRELOAD;
                    }
                    else if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        follower.followPath(goToScore3);
                        CS = STATES.PRELOAD;
                    }
                    break;
                case BACK_TO_WALL: //vine de la submersible la urmatorul specimen de pe perete, apoi repeta pasii pentru colectarea si punctarea specimenului
                    intakeSubsystem.goToWall();
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToWall2, true);
                    CS = STATES.COLLECTING;
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        SCORING_CS = SCORING_STATES.SPECIMEN2;
                    }else if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        SCORING_CS = SCORING_STATES.SPECIMEN3;
                    }
                    break;
                case PARK:
                    follower.setMaxPower(maxSpeed);
                    follower.followPath(goToPark);
                    CS = STATES.MOVING;
                    NS = STATES.PARKED;
                    break;
                case PARKED:
                    break;
            }


            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.update();

            robotSystems.update();
            follower.update();
        }
    }
}
