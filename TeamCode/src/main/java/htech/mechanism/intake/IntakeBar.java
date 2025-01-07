package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsIntake;
import htech.config.Servos;

public class IntakeBar {
    private final Servo barServo;
    private final Servo barServo2;
    private double currentPosition;

    public IntakeBar(HardwareMap hardwareMap) {
        barServo = hardwareMap.get(Servo.class, Servos.intakeBarServoLeft);
        barServo2 = hardwareMap.get(Servo.class, Servos.intakeBarServoRight);
        barServo.setPosition(PositionsIntake.transferPositionBar + PositionsIntake.offsetBar);
        barServo2.setPosition(PositionsIntake.transferPositionBar);
        currentPosition = PositionsIntake.transferPositionBar;
    }

    public void goToGround() {
        barServo.setPosition(PositionsIntake.groundPositionBar + PositionsIntake.offsetBar);
        barServo2.setPosition(PositionsIntake.groundPositionBar);
        currentPosition = PositionsIntake.groundPositionBar;
    }

    public void goToWall() {
        barServo.setPosition(PositionsIntake.wallPositionBar + PositionsIntake.offsetBar);
        barServo2.setPosition(PositionsIntake.wallPositionBar);
        currentPosition = PositionsIntake.wallPositionBar;
    }

    public void goToTransfer() {
        barServo.setPosition(PositionsIntake.transferPositionBar + PositionsIntake.offsetBar);
        barServo2.setPosition(PositionsIntake.transferPositionBar);
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
        barServo.setPosition(PositionsIntake.collectPositionBar + PositionsIntake.offsetBar);
        barServo2.setPosition(PositionsIntake.collectPositionBar);
        currentPosition = PositionsIntake.collectPositionBar;
    }

    public void update() {
        barServo.setPosition(currentPosition);
        barServo2.setPosition(currentPosition);
    }

}
