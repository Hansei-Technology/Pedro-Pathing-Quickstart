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
import org.firstinspires.ftc.teamcode.htech.subsystem.RobotSystems;

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
    RobotSystems robotSystems;


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

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);

        waitForStart();

        matchTimer.reset();

        while (opModeIsActive()) {

            chassisMovement.move(gamepad1);

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
            //transfer
            if(gamepad2.x) {
                robotSystems.transferState = RobotSystems.TransferStates.LIFT_GOING_DOWN;
            }

            if (stickyGamepad2.right_bumper) intakeSubsystem.claw.toggle();
            if (stickyGamepad2.left_bumper) intakeSubsystem.rotation.togglePerpendicular();


            if(gamepad2.dpad_down) extendo.goToGround();
            if(gamepad2.dpad_up) extendo.goToMax();

            //lift control
            if(gamepad1.b) {
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
            }
            if(gamepad1.y) {
                lift.goToHighBasket();
                outtakeSubsystem.goToSampleScore();
            }
            if(gamepad1.a) {
                lift.goToGround();
                outtakeSubsystem.goToTransfer();
                if(intakeSubsystem.CS == IntakeSubsystem.IntakeState.TRANSFER) intakeSubsystem.goToReady();
            }
            if(gamepad1.dpad_left) outtakeSubsystem.goToSampleScore();

            if(gamepad1.right_bumper) outtakeSubsystem.claw.open();
            if(gamepad1.left_bumper) lift.goToMagicPos();


            if(matchTimer.seconds() > 90) { //only in endgame
                //hang.setPower(-gamepad2.right_stick_y);
            }



            stickyGamepad2.update();
            stickyGamepad1.update();
            robotSystems.update();

            //telemetry:
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.CS);

            telemetry.update();
        }
    }

}
