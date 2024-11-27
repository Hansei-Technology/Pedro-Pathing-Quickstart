package org.firstinspires.ftc.teamcode.htech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.subsystem.OuttakeSubsystem;

@TeleOp(group = "HTech", name = "[TELEOP] Outtake Teleop")
public class OuttakeTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(hardwareMap);

        String positionName = "preScore";

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("[STATUS]", "Current Position: " + positionName);

            if (gamepad1.dpad_up) {
                positionName = "sampleScore";
                outtakeSubsystem.goToSampleScore();
            } else if (gamepad1.dpad_left) {
                positionName = "specimenScore";
                outtakeSubsystem.goToSpecimenScore();
            } else if (gamepad1.dpad_down) {
                positionName = "transfer";
                outtakeSubsystem.goToTransfer();
            }

            telemetry.update();
        }
    }
}
