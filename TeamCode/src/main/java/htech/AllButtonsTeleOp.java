package htech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.classes.StickyGamepad;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;

@TeleOp(name = "[TELEOP] AllButtons", group = "HTech")
public class AllButtonsTeleOp extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        // SUBSYSTEMS //
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();

        // CLASSES //
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1, this);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "Main Teleop is running.");

            chassisMovement.updateMovement(gamepad1);

            if (stickyGamepad.left_bumper) intakeSubsystem.rotation.togglePerpendicular();
            if (stickyGamepad.right_bumper) intakeSubsystem.claw.toggle();
            if (stickyGamepad.dpad_right) outtakeSubsystem.claw.toggle();

            if (gamepad1.x) {
                intakeSubsystem.goDown();
            } else if (gamepad1.a) {
                intakeSubsystem.goToWall();
            } else if (gamepad1.b) {
                intakeSubsystem.goToReady();

                outtakeSubsystem.goToTransfer();
            } else if (gamepad1.y) {
                intakeSubsystem.goToTransfer();
            }

            if (gamepad1.dpad_left) {
                outtakeSubsystem.goToSpecimenScore();
            } else if (gamepad1.dpad_up) {
                outtakeSubsystem.goToSampleScore();
            }


            if(gamepad2.a) lift.goToGround();
            if(gamepad2.y) lift.goToHighChamber();

            if(gamepad2.dpad_down) extendo.goToGround();
            if(gamepad2.dpad_up) extendo.goToMax();

            stickyGamepad.update();
            lift.update();
            extendo.update();
            telemetry.update();
        }
    }



}
