package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.classes.StickyGamepad;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;

@Disabled
@TeleOp(name = "[TELEOP] SOLO", group = "HTech")
public class TeleOpSolo extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    RobotSystems robotSystems;
//    ChassisFollower chassisFollower;

    @Override
    public void runOpMode() throws InterruptedException {
        // SUBSYSTEMS //
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        chassisFollower = new ChassisFollower(hardwareMap);

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);

        waitForStart();

        intakeSubsystem.init();
        outtakeSubsystem.init();

        matchTimer.reset();

        //lift.goToMinusPark();
        //lift.reset();
        while (opModeIsActive()) {

            chassisMovement.move(gamepad1);
//            chassisFollower.move(gamepad1);

            extendo.moveFree(gamepad1.right_trigger - gamepad1.left_trigger);



            stickyGamepad2.update();
            stickyGamepad1.update();
            robotSystems.update();

            //reset lift
            if(gamepad1.dpad_down) {
                while (gamepad1.left_stick_button) {
                    lift.setPower(-0.35);
                }
                lift.setPower(0);
                lift.reset(gamepad1);
                gamepad1.rumble(100);
            }

            //telemetry:
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.intakeState);
            telemetry.addData("intakeTimer", robotSystems.timerCollect.milliseconds());
            telemetry.addData("extendoPID", extendo.pidEnabled);
            telemetry.addData("outtakeRot", robotSystems.outtakeSubsystem.joint.getRot());

            telemetry.update();
        }
    }

}
