package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
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

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "BasketAuto", group = "HTECH")
public class BasketAuto extends OpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;


    private Telemetry telemetryA;

    private Follower follower;

    private Path goToPreload;

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -26, PRELOAD_Y = 0, PRELOAD_ANGLE;
    public static double SAFE_X = -7, SAFE_Y = -10, SAFE_ANGLE;
    public static double SAFE_BASKET_X = -20, SAFE_BASKET_Y = -10, SAFE_BASKET_ANGLE;
    public static double SAMPLE1_X = -30.5, SAMPLE1_Y = -37, SAMPLE1_ANGLE = 180;
    public static double SAMPLE2_X = -39.5, SAMPLE2_Y = -39.5, SAMPLE2_ANGLE = 270;
    public static double SAMPLE3_X = -39.3, SAMPLE3_Y = -39.5, SAMPLE3_ANGLE = 270;
    public static double BASKET1_X = -13, BASKET1_Y = -38.5, BASKET1_ANGLE = 120;
    public static double BASKET2_X = -13, BASKET2_Y = -38.5, BASKET2_ANGLE = 123;
    public static double BASKET3_X = -13, BASKET3_Y = -38.5, BASKET3_ANGLE = 125;
    public static double PARK_X = -55, PARK_Y = -14.3, PARK_ANGLE = 270;
    public static double SAFE_PARK_X = -52, SAFE_PARK_Y = -38, SAFE_PARK_ANGLE;

    public static int timeToPreload = 400;
    public static int timeToSample = 200;
    public static int timeToCollect1 = 1200;
    public static int timeToCollect2 = 1200;
    public static int timeToCollect3 = 1500;
    public static int time_to_transfer = 600;
    public static int time_to_lift = 850;
    public static int time_to_drop = 1050;


    public static int extendoPoz3 = 350;

    PathChain goTo1Sample;
    PathChain goTo2Sample;
    PathChain goTo3Sample;
    PathChain goTo1Basket;
    PathChain goTo2Basket;
    PathChain goTo3Basket;
    PathChain goToPark;


    private enum STATES {
        MOVING,
        WAITING,
        TRANSFERING,
        PRELOAD,
        PLACING_PRELOAD,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        BASKET1,
        BASKET2,
        BASKET3,
        COLLECTING1,
        COLLECTING2,
        COLLECTING3,
        PARK,
        PARKED
    }
    public STATES CS = STATES.PRELOAD, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;

    Pose Sample1 = new Pose(SAMPLE1_X, SAMPLE1_Y, SAMPLE1_ANGLE);
    Pose Sample2 = new Pose(SAMPLE2_X, SAMPLE2_Y, SAMPLE2_ANGLE);
    Pose Sample3 = new Pose(SAMPLE3_X, SAMPLE3_Y, SAMPLE3_ANGLE);
    Pose Basket1 = new Pose(BASKET1_X, BASKET1_Y, BASKET1_ANGLE);
    Pose Basket2 = new Pose(BASKET2_X, BASKET2_Y, BASKET2_ANGLE);
    Pose Basket3 = new Pose(BASKET3_X, BASKET3_Y, BASKET3_ANGLE);

    @Override
    public void init() {
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        intakeSubsystem.init();
        outtakeSubsystem.init();

        outtakeSubsystem.claw.close();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        //follower.setPose(new Pose(START_X, START_Y, START_ANGLE));

        goToPreload = new Path(new BezierLine(new Point(START_X,START_Y, Point.CARTESIAN), new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)));
        goToPreload.setConstantHeadingInterpolation(PRELOAD_ANGLE);
        goToPreload.setReversed(true);
        follower.setMaxPower(0.6);


        goTo1Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN),
                                new Point(SAFE_X, SAFE_Y, Point.CARTESIAN),
                                new Point(SAMPLE1_X,SAMPLE1_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(START_ANGLE), Math.toRadians(SAMPLE1_ANGLE))
                .setPathEndTimeoutConstraint(500)

                .build();


        goTo1Basket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                                //new Point(SAFE_BASKET_X, SAFE_BASKET_Y, Point.CARTESIAN),
                                new Point(BASKET1_X, BASKET1_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE1_ANGLE), Math.toRadians(BASKET1_ANGLE))
                //.setPathEndTimeoutConstraint(500)
                //.setReversed(true)
                .build();


        goTo2Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(BASKET1_X,BASKET1_Y, Point.CARTESIAN),
                                new Point(SAMPLE2_X,SAMPLE2_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(BASKET1_ANGLE), Math.toRadians(SAMPLE2_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();


        goTo2Basket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_X, SAMPLE2_Y, Point.CARTESIAN),
                                new Point(BASKET2_X, BASKET2_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE2_ANGLE), Math.toRadians(BASKET2_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();


        goTo3Sample = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(BASKET2_X,BASKET2_Y, Point.CARTESIAN),
                                new Point(SAMPLE3_X,SAMPLE3_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(BASKET2_ANGLE), Math.toRadians(SAMPLE3_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();


        goTo3Basket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE3_X, SAMPLE3_Y, Point.CARTESIAN),
                                new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(SAMPLE3_ANGLE), Math.toRadians(BASKET3_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();

        goToPark = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(BASKET3_X, BASKET3_Y, Point.CARTESIAN),
                                new Point(SAFE_PARK_X, SAFE_PARK_Y, Point.CARTESIAN),
                                new Point(PARK_X, PARK_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(BASKET3_ANGLE), Math.toRadians(PARK_ANGLE))
                .setPathEndTimeoutConstraint(500)
                .build();


        follower.followPath(goToPreload);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    boolean firstTime = true;

    @Override
    public void loop() {
        follower.update();
        robotSystems.update();


        switch (CS) {
            case PRELOAD:
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
                NS = STATES.PLACING_PRELOAD;
                CS = STATES.MOVING;
                break;

            case MOVING:
                if(!follower.isBusy() ||  follower.getCurrentTValue() > 0.99) {
                    switch(NS) {
                        case PLACING_PRELOAD:
                            lift.goToMagicPos();
                            break;
                        case BASKET1:
                            lift.goToHighBasket();
                            break;
                    }
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
                        //lift.goToHighBasket();
//                        outtakeSubsystem.goToSampleScore();
                        firstTime = true;
                        CS = STATES.MOVING;
                    }
                }
                break;

            case PLACING_PRELOAD:
                if(lift.isAtPosition()) {
                    outtakeSubsystem.claw.open();
                    NS = STATES.SAMPLE1;
                    CS = STATES.WAITING;
                    TIME_TO_WAIT = timeToPreload;
                    timer.reset();
                }
                break;

            case SAMPLE1:
                follower.setMaxPower(0.6);
                follower.followPath(goTo1Sample, true);
                outtakeSubsystem.goToTransfer();
                lift.goToGround();
                intakeSubsystem.goDown();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING1;
                CS = STATES.MOVING;
                break;

            case COLLECTING1:
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    follower.setMaxPower(0.6);
                    follower.followPath(goTo1Basket, true);
                    //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN; //start transfer
                    NS = STATES.BASKET1;
                    firstTime = true;
                    CS = STATES.TRANSFERING;
                }
                if(timer.milliseconds() > timeToCollect1 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET1:
                //lift.goToHighBasket();
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                }

                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.SAMPLE2;
                    CS = STATES.WAITING;
                }
                break;

            case SAMPLE2:
                follower.setMaxPower(0.6);
                follower.followPath(goTo2Sample, true);
                outtakeSubsystem.goToTransfer();
                lift.goToGround();
                intakeSubsystem.goDown();
                intakeSubsystem.rotation.goToPerpendicular();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING2;
                CS = STATES.MOVING;
                break;

            case COLLECTING2:
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    follower.setMaxPower(0.6);
                    follower.followPath(goTo2Basket, true);
                    //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN;
                    NS = STATES.BASKET2;
                    firstTime = true;
                    CS = STATES.TRANSFERING;
                }
                if(timer.milliseconds() > timeToCollect2 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET2:
                lift.goToHighBasket();
                    if(firstTime) {
                        timer.reset();
                        firstTime = false;
                    }
                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.SAMPLE3;
                    CS = STATES.WAITING;
                }
                break;

            case SAMPLE3:
                follower.setMaxPower(0.6);
                follower.followPath(goTo3Sample, true);
                outtakeSubsystem.goToTransfer();
                lift.goToGround();
                intakeSubsystem.goDown();
                intakeSubsystem.rotation.goToPerpendicular();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.COLLECTING3;
                CS = STATES.MOVING;
                break;

            case COLLECTING3:
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                    extendo.goToPos(extendoPoz3);
                }
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.COLECT_GOING_UP || intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.WALL) {
                    follower.setMaxPower(0.6);
                    follower.followPath(goTo3Basket, true);
                    //robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN;
                    NS = STATES.BASKET3;
                    firstTime = true;
                    CS = STATES.TRANSFERING;
                }
                if(timer.milliseconds() > timeToCollect3 && intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.collect(true);
                    timer.reset();
                }
                break;

            case BASKET3:
                lift.goToHighBasket();
                if(firstTime) {
                    timer.reset();
                    firstTime = false;
                }

                if(timer.milliseconds() > time_to_lift) {
                    outtakeSubsystem.goToSampleScore();
                }

                if(timer.milliseconds() > time_to_drop) {
                    outtakeSubsystem.claw.open();
                    timer.reset();
                    TIME_TO_WAIT = timeToSample;
                    NS = STATES.PARK;
                    CS = STATES.WAITING;
                }
                break;

            case PARK:
                follower.setMaxPower(0.8);

                follower.followPath(goToPark, true);
                outtakeSubsystem.goToTransfer();
                lift.goToGround();
                intakeSubsystem.goToReady();
                intakeSubsystem.claw.open();

                timer.reset();
                NS = STATES.PARKED;
                CS = STATES.MOVING;
                break;

            case PARKED:
                //lift.goToGround();
                lift.goToPark();
                break;

        }

        telemetry.addData("Match Time", matchTimer.seconds());
        telemetry.addData("Current State", CS);
        telemetry.addData("Next State", follower.getPose());
        telemetry.addData("Transfer State", robotSystems.transferState);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("extendoTarget", extendo.target_position);
        telemetry.update();

        //follower.telemetryDebug(telemetryA);
    }


    public boolean isAtPos(Pose current, Pose target, double tolerance) { //i dont know if this works
        return Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance;
    }
}
