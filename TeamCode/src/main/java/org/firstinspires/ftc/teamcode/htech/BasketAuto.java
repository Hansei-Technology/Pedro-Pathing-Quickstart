package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.htech.MainTeleOp;
import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;
import org.firstinspires.ftc.teamcode.htech.subsystem.ChassisMovement;
import org.firstinspires.ftc.teamcode.htech.subsystem.ExtendoSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.LiftSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

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
@Autonomous (name = "BasketAuto", group = "AUTO")
public class BasketAuto extends OpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;


    private Telemetry telemetryA;

    private Follower follower;

    private Path goToPreload;
    private Path goTo1Sample;
    private Path goTo2Sample;
    private Path goTo3Sample;
    private Path goTo1Basket;
    private Path goTo2Basket;
    private Path goTo3Basket;
    private Path goToPark;

    public static double START_X = 0, START_Y = 0, START_ANGLE = 0;
    public static double PRELOAD_X = -21, PRELOAD_Y = 0, PRELOAD_ANGLE;
    public static double SAFE_X = -7, SAFE_Y = -20, SAFE_ANGLE;
    public static double SAMPLE1_X = -10, SAMPLE1_Y = -36, SAMPLE1_ANGLE = 180;
    public static double SAMPLE2_X, SAMPLE2_Y, SAMPLE2_ANGLE;
    public static double SAMPLE3_X, SAMPLE3_Y, SAMPLE3_ANGLE;
    public static double BASKET1_X, BASKET1_Y, BASKET1_ANGLE;
    public static double BASKET2_X, BASKET2_Y, BASKET2_ANGLE;
    public static double BASKET3_X, BASKET3_Y, BASKET3_ANGLE;
    public static double PARK_X, PARK_Y, PARK_ANGLE;

    public static int timeToPreload = 500;


    private enum STATES {
        MOVING,
        WAITING,
        PRELOAD,
        PLACING_PRELOAD,
        SAMPLE1,
        SAMPLE2,
        SAMPLE3,
        BASKET1,
        BASKET2,
        BASKET3,
        PARK
    }
    public STATES CS = STATES.PRELOAD, PS = STATES.MOVING, NS = STATES.MOVING;
    public int TIME_TO_WAIT = 0;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        outtakeSubsystem.claw.close();

        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.6);
        //follower.setPose(new Pose(START_X, START_Y, START_ANGLE));

        goToPreload = new Path(new BezierLine(new Point(START_X,START_Y, Point.CARTESIAN), new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN)));
        goToPreload.setConstantHeadingInterpolation(PRELOAD_ANGLE);
        goToPreload.setReversed(true);

        goTo1Sample = new Path(new BezierLine(new Point(PRELOAD_X,PRELOAD_Y, Point.CARTESIAN), new Point(SAMPLE1_X,SAMPLE1_Y, Point.CARTESIAN)));
        goTo1Sample.setLinearHeadingInterpolation(Math.toRadians(START_ANGLE), Math.toRadians(SAMPLE1_ANGLE));
        goTo1Sample.setReversed(false);

        follower.followPath(goToPreload);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        lift.update();
        extendo.update();
        intakeSubsystem.updateColect();


        switch (CS) {
            case PRELOAD:
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
                PS = STATES.PRELOAD;
                CS = STATES.MOVING;
                break;

            case MOVING:
                switch (PS) {
                    case PRELOAD:
                        if(!follower.isBusy()) {
                            lift.goToMagicPos();
                            PS = STATES.MOVING;
                            CS = STATES.PLACING_PRELOAD;
                        }
                        break;
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
            case WAITING:
                if(timer.milliseconds() > TIME_TO_WAIT) {
                    CS = NS;
                }
                break;
            case SAMPLE1:
                follower.followPath(goTo1Sample);
                timer.reset();
                PS = STATES.PLACING_PRELOAD;
                CS = STATES.MOVING;
                break;
        }

        telemetry.addData("Match Time", matchTimer.seconds());
        follower.telemetryDebug(telemetryA);
    }




    enum TransferStates {
        IDLE,
        LIFT_GOING_DOWN,
        INTAKE_DOWN,
        INTAKE_WALL,
        READY_TO_TRANSFER,
        WAITING_TO_FALL,
        TRANSFER_READY,
        LIFT_GOING_UP,
    }
    TransferStates transferState = TransferStates.IDLE;

    void updateTranfer() {
        switch (transferState) {
            case IDLE:
                break;

            case LIFT_GOING_DOWN:
                //on entry
                lift.goToGround();
                extendo.goToGround();
                outtakeSubsystem.goToTransfer();
                outtakeSubsystem.claw.open();
                timer.reset();

                //condition to exit
                if (intakeSubsystem.CS == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_DOWN;
                } else {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_WALL;
                }
                break;

            case INTAKE_DOWN:
                if (lift.isDown() && extendo.isDown() && timer.milliseconds() > RobotSettings.timeDown_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case INTAKE_WALL:
                if (lift.isDown() && extendo.isDown() && timer.milliseconds() > RobotSettings.timeWall_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case READY_TO_TRANSFER:
                if (timer.milliseconds() > RobotSettings.timeReady_Transfer) {
                    timer.reset();
                    intakeSubsystem.claw.open();
                    intakeSubsystem.claw.open();
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.WAITING_TO_FALL;
                }
                break;

            case WAITING_TO_FALL:
                if (timer.milliseconds() > RobotSettings.timeToDropElement) {
                    timer.reset();
                    intakeSubsystem.goToWall();
                    transferState = TransferStates.TRANSFER_READY;
                }
                break;
            case TRANSFER_READY:
                if (timer.milliseconds() > RobotSettings.timeToCloseOuttake) {
                    timer.reset();

                    transferState = TransferStates.IDLE;
                }
        }
    }
}
