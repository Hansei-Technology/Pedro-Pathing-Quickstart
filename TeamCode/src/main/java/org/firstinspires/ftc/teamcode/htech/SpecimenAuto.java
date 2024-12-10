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
    IntakeClaw cleste;

    private Follower follower;
    private Path safeScoring;
    private Path goToPreload;
    private PathChain goTo1Sample;
    private PathChain goTo2Sample;
    private PathChain goTo3Sample;
    private PathChain goTo1Specimen;
    private PathChain goTo2Specimen;
    private PathChain goTo3Specimen;
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

    public static int timeToPreload = 200;
    public static int timeToSample = 200;
    public static int timeToCollect1 = 1000;
    public static int timeToCollect2 = 1000;
    public static int timeToCollect3 = 1000;
    public static int time_to_transfer = 500;
    public static int time_to_lift = 650;
    public static int time_to_drop = 800;

    //start pose 135, 83

    public static double START_X = 135, START_Y = 86, START_ANGLE = -180;
    public static double PRELOAD_X = 108.5, PRELOAD_Y = 86, PRELOAD_ANGLE = START_ANGLE;

    public static double SAFE1_X = 135, SAFE1_Y = 94.3, SAFE11_ANGLE;
    public static double SAMPLE1_X = 87, SAMPLE1_Y = 119, SAMPLE1_ANGLE = -180;
    public static double HUMAN1_X = 130, HUMAN1_Y = 119, HUMAN1_ANGLE = -180;

    public static double SAFE2_X = 87, SAFE2_Y = 108, SAFE2_ANGLE;
    public static double SAMPLE2_X = 87, SAMPLE2_Y = 130, SAMPLE2_ANGLE = -180;
    public static double HUMAN2_X = 130, HUMAN2_Y = 130, HUMAN2_ANGLE = -180;

    public static double SAFE3_X = 95, SAFE3_Y = 128, SAFE3_ANGLE;
    public static double SAMPLE3_X = 87, SAMPLE3_Y = 140, SAMPLE3_ANGLE = -180;
    public static double HUMAN3_X = 130, HUMAN3_Y = 140, HUMAN3_ANGLE = -180;
    public static double SAFE4_X = 130, SAFE4_Y = 120, SAFE4_ANGLE = 0;

    public static double SAFE_SPECIMEN_X = -20, SAFE_SPECIMEN_Y = -10, SAFE_SPECIMEN_ANGLE;

    public static double SPECIMEN1_X = 140, SPECIMEN1_Y = 120, SPECIMEN1_ANGLE = 0;
    public static double SPECIMEN2_X = -14, SPECIMEN2_Y = -38.5, SPECIMEN2_ANGLE = 123;
    public static double SPECIMEN3_X = -14, SPECIMEN3_Y = -38.5, SPECIMEN3_ANGLE = 125;
    public static double PARK_X = 140, PARK_Y = 120, PARK_ANGLE = 90;
    public static double SAFE_PARK_X = -52, SAFE_PARK_Y = -38, SAFE_PARK_ANGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cleste = new IntakeClaw(hardwareMap);
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        follower = new Follower(hardwareMap);
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

        goTo1Specimen = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAFE4_X, SAFE4_Y, Point.CARTESIAN),
                                new Point(SPECIMEN1_X, SPECIMEN1_Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(Math.toRadians(PRELOAD_ANGLE)))
                .setPathEndTimeoutConstraint(500)
                .build();

        goTo2Specimen = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(SPECIMEN2_X, SPECIMEN2_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE), Math.toRadians(SPECIMEN2_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goTo3Specimen = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(SPECIMEN3_X, SPECIMEN3_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE), Math.toRadians(SPECIMEN3_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(PRELOAD_X, PRELOAD_Y, Point.CARTESIAN),
                                new Point(PARK_X, PARK_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SPECIMEN3_ANGLE), Math.toRadians(PARK_ANGLE))
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
                    if(!follower.isBusy() ||  follower.getCurrentTValue() > 0.99 ) {
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
                            lift.goToHighChamber();
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

                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        CS = STATES.SAFE_SCORE;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN2){
                        CS = STATES.SAFE_SCORE;
                        SCORING_CS = SCORING_STATES.SPECIMEN3;
                        break;
                    }
                    if(SCORING_CS == SCORING_STATES.SPECIMEN3){
                        CS = STATES.PARK;
                    }
                    break;
                case SAMPLE1:
                    follower.setMaxPower(0.6);
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
                    follower.setMaxPower(0.6);
                    follower.followPath(goTo2Sample);

                    NS = STATES.SAMPLE3;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAMPLE3:
                    follower.setMaxPower(0.6);
                    follower.followPath(goTo3Sample);

                    NS = STATES.SAFE4;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case SAFE4:
                    follower.setMaxPower(0.6);
                    follower.followPath(goToSafe4, true);
                    NS = STATES.SPECIMEN1;
                    firstTime = true;
                    CS = STATES.MOVING;
                    break;
                case COLLECTING1:
                    follower.setMaxPower(0.4);
                    follower.followPath(goTo1Specimen);
                    if(SCORING_CS == SCORING_STATES.IDLE){
                        SCORING_CS = SCORING_STATES.SPECIMEN1;
                    }
                    NS = STATES.COLLECTING2;
                    CS = STATES.MOVING;
                    break;
                case COLLECTING2:
                    cleste.close();
                    robotSystems.transferState = RobotSystems.TransferStates.INTAKE_WALL;
                    CS = STATES.TRANSFERING;
                    NS = STATES.SCORE1;
                    break;
                case SCORE1:
                    follower.setMaxPower(0.6);
                    follower.followPath(goToPreload);
                    CS = STATES.PRELOAD;
                    break;
                case SAFE_SCORE:
                    intakeSubsystem.goToWall();
                    follower.setMaxPower(0.6);
                    follower.followPath(safeScoring, true);
                    CS = STATES.COLLECTING1;
                    if(SCORING_CS == SCORING_STATES.SPECIMEN1){
                        SCORING_CS = SCORING_STATES.SPECIMEN2;
                    }
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
