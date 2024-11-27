package org.firstinspires.ftc.teamcode.htech.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.mechanism.HangSystem;

@TeleOp
public class HangTest extends LinearOpMode {
    HangSystem hang;
    @Override
    public void runOpMode() throws InterruptedException {
        hang = new HangSystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            hang.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}
