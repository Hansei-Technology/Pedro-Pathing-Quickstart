package org.firstinspires.ftc.teamcode.htech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.classes.StickyGamepad;
import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;
import org.firstinspires.ftc.teamcode.htech.mechanism.HangSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.ChassisMovement;
import org.firstinspires.ftc.teamcode.htech.subsystem.ExtendoSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.LiftSystem;
import org.firstinspires.ftc.teamcode.htech.subsystem.OuttakeSubsystem;

@TeleOp(name = "[TELEOP] Main", group = "HTech")
public class MainTeleOp extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;


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

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);

        waitForStart();

        matchTimer.reset();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());

            chassisMovement.updateMovement(gamepad1);

            if(gamepad1.right_trigger > 0.1){
                RobotSettings.rotationSpeed = 0.5;
            }

            if(gamepad1.left_trigger > 0.1){
                chassisMovement.updateMovementReverse(gamepad1);
            }

            //intake
            if (gamepad2.a) {
                intakeSubsystem.goDown();
            }
            if(stickyGamepad2.b) {
                intakeSubsystem.collect();
            }
            if (gamepad2.y) {
                intakeSubsystem.goToWall();
            }
            if (stickyGamepad2.right_bumper) intakeSubsystem.claw.toggle();
            if (stickyGamepad2.left_bumper) intakeSubsystem.rotation.togglePerpendicular();


            if(gamepad2.dpad_down) extendo.goToGround();
            if(gamepad2.dpad_up) extendo.goToMax();

            //lift control
            if(gamepad1.y) {
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
            }
            if(gamepad1.b) {
                lift.goToMagicPos();
            }
            if(gamepad1.a) {
                lift.goToGround();
                outtakeSubsystem.goToTransfer();
                if(intakeSubsystem.CS == IntakeSubsystem.IntakeState.TRANSFER) intakeSubsystem.goToReady();
            }
            if(gamepad1.dpad_left) outtakeSubsystem.goToSampleScore();

            //transfer
            if(gamepad1.x) {
                transferState = TransferStates.LIFT_GOING_DOWN;
            }

            if(gamepad1.right_bumper) outtakeSubsystem.claw.open();


            if(matchTimer.seconds() > 90) { //only in endgame
                //hang.setPower(-gamepad2.right_stick_y);
            }



            stickyGamepad2.update();
            stickyGamepad1.update();
            intakeSubsystem.updateColect();
            lift.update();
            extendo.update();
            updateTranfer();

            //telemetry:
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.CS);

            telemetry.update();
        }
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
                if(intakeSubsystem.CS == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_DOWN;
                } else {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_WALL;
                }
                break;

            case INTAKE_DOWN:
                if(lift.isDown() && extendo.isDown() && timer.milliseconds() > RobotSettings.timeDown_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case INTAKE_WALL:
                if(lift.isDown() && extendo.isDown() && timer.milliseconds() > RobotSettings.timeWall_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case READY_TO_TRANSFER:
                if(timer.milliseconds() > RobotSettings.timeReady_Transfer) {
                    timer.reset();
                    intakeSubsystem.claw.open();
                    intakeSubsystem.claw.open();
                    transferState = TransferStates.WAITING_TO_FALL;
                }
                break;

            case WAITING_TO_FALL:
                if(timer.milliseconds() > RobotSettings.timeToDropElement) {
                    timer.reset();
                    intakeSubsystem.goToWall();
                    transferState = TransferStates.TRANSFER_READY;
                }
                break;
            case TRANSFER_READY:
                if(timer.milliseconds() > RobotSettings.timeToCloseOuttake) {
                    timer.reset();
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.IDLE;
                }
        }
    }

}
