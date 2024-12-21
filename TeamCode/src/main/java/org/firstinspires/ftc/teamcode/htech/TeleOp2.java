package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.classes.StickyGamepad;
import org.firstinspires.ftc.teamcode.htech.subsystem.ChassisFollower;
import org.firstinspires.ftc.teamcode.htech.subsystem.ChassisMovement;
import org.firstinspires.ftc.teamcode.htech.subsystem.ExtendoSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.LiftSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.RobotSystems;

@TeleOp(name = "[TELEOP] 2", group = "HTech")
public class TeleOp2 extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    RobotSystems robotSystems;
    ChassisFollower chassisFollower;

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
        chassisFollower = new ChassisFollower(hardwareMap);

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);

        waitForStart();

        intakeSubsystem.init();
        outtakeSubsystem.init();

        matchTimer.reset();

        while (opModeIsActive()) {

            chassisMovement.move(gamepad1);
//            chassisFollower.move(gamepad1);

            //extendo
            extendo.moveFree(gamepad1.right_trigger - gamepad1.left_trigger);
            if(gamepad2.dpad_up) {
                extendo.goToPos(230);
                intakeSubsystem.goDown();
            }


            //intake
            if(stickyGamepad1.right_bumper) {
                intakeSubsystem.collect(false);
            }
            if (stickyGamepad1.left_bumper) intakeSubsystem.claw.toggle();

            //rotations(both of them)
            if(extendo.isDown()) { //handle outtake rotation
                if(stickyGamepad2.dpad_right) {
                    robotSystems.outtakeSubsystem.joint.rotateRight();
                }
                if(stickyGamepad2.dpad_left) {
                    robotSystems.outtakeSubsystem.joint.rotateLeft();
                }
            } else {
                intakeSubsystem.rotation.handleRotation(gamepad2);
            }


            //lift
            if(gamepad2.b) {
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
            }
            if(gamepad2.y) {
                lift.goToHighBasket();
                outtakeSubsystem.goToSampleScore();
            }
            if(gamepad2.a) {
                lift.goToGround();
                outtakeSubsystem.goToTransfer();
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.TRANSFER) intakeSubsystem.goToReady();
            }

            if(gamepad2.right_bumper) outtakeSubsystem.claw.open();
            if(gamepad2.left_bumper) robotSystems.scoreSpecimen();


            stickyGamepad2.update();
            stickyGamepad1.update();
            robotSystems.update();

            //reset lift
            if(gamepad2.dpad_down) {
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
