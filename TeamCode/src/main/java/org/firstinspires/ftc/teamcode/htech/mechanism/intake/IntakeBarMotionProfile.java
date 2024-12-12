package org.firstinspires.ftc.teamcode.htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.classes.DualServoController;
import org.firstinspires.ftc.teamcode.htech.config.PositionsIntake;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class IntakeBarMotionProfile {
    private final Servo barServo;
    private final Servo barServo2;
    private double currentPosition;
    DualServoController dualServoController;

    public IntakeBarMotionProfile(HardwareMap hardwareMap) {
        barServo = hardwareMap.get(Servo.class, Servos.intakeBarServoLeft);
        barServo2 = hardwareMap.get(Servo.class, Servos.intakeBarServoRight);
//        barServo.setPosition(PositionsIntake.transferPositionBar);
//        barServo2.setPosition(PositionsIntake.transferPositionBar);

        dualServoController = new DualServoController(barServo, barServo2);
        dualServoController.setTargetPosition(PositionsIntake.transferPositionBar);

//        currentPosition = PositionsIntake.transferPositionBar;
    }

    public void goToGround() {
        dualServoController.setTargetPosition(PositionsIntake.groundPositionBar);
        currentPosition = PositionsIntake.groundPositionBar;
    }

    public void goToWall() {
        dualServoController.setTargetPosition(PositionsIntake.wallPositionBar);
        currentPosition = PositionsIntake.wallPositionBar;
    }

    public void goToTransfer() {
        dualServoController.setTargetPosition(PositionsIntake.transferPositionBar);
        currentPosition = PositionsIntake.transferPositionBar;
    }

    public void cycleHeight() {
        if (currentPosition == PositionsIntake.groundPositionBar) {
            this.goToWall();
        } else if (currentPosition == PositionsIntake.wallPositionBar) {
            this.goToGround();
        }
    }

    public void goToCollect() {
        dualServoController.setTargetPosition(PositionsIntake.collectPositionBar);
        currentPosition = PositionsIntake.collectPositionBar;
    }

    public void update() {
        dualServoController.update();
    }
}
