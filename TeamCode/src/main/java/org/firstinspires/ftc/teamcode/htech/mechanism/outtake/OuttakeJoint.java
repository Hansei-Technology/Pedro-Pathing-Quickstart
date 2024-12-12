package org.firstinspires.ftc.teamcode.htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.PositionsOuttake;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class OuttakeJoint {
    private final Servo servoLeft;
    private final Servo servoRight;

    int rotLevel = 0;


    private double currentPositionLeft;
    private double currentPositionRight;

    public OuttakeJoint(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, Servos.outtakeLeft);
        servoRight = hardwareMap.get(Servo.class, Servos.outtakeRight);

//        servoLeft.setPosition(PositionsOuttake.jointTransferLeft + PositionsOuttake.jointRotation90 * rotLevel);
//        servoRight.setPosition(PositionsOuttake.jointTransferRight + PositionsOuttake.jointRotation90 * rotLevel);

        currentPositionLeft = PositionsOuttake.jointTransferLeft;
        currentPositionRight = PositionsOuttake.jointTransferRight;
    }

    public void goToTransfer() {
        servoLeft.setPosition(PositionsOuttake.jointTransferLeft + PositionsOuttake.jointRotation90 * rotLevel);
        servoRight.setPosition(PositionsOuttake.jointTransferRight + PositionsOuttake.jointRotation90 * rotLevel);

        currentPositionLeft = PositionsOuttake.jointTransferLeft;
        currentPositionRight = PositionsOuttake.jointTransferRight;
    }

    public void goToSpecimenScore() {
        servoLeft.setPosition(PositionsOuttake.jointSpecimenLeft + PositionsOuttake.jointRotation90 * rotLevel);
        servoRight.setPosition(PositionsOuttake.jointSpecimenRight + PositionsOuttake.jointRotation90 * rotLevel);

        currentPositionLeft = PositionsOuttake.jointSpecimenLeft;
        currentPositionRight = PositionsOuttake.jointSpecimenRight;
    }

    public void goToBasketScore() {
        servoLeft.setPosition(PositionsOuttake.jointBasketLeft + PositionsOuttake.jointRotation90 * rotLevel);
        servoRight.setPosition(PositionsOuttake.jointBasketRight + PositionsOuttake.jointRotation90 * rotLevel);

        currentPositionLeft = PositionsOuttake.jointBasketLeft;
        currentPositionRight = PositionsOuttake.jointBasketRight;
    }

    public int getRot() {
        return 90 * rotLevel;
    }

    public void rotateLeft() {
        rotLevel++;
        rotLevel %= 4;

        servoLeft.setPosition(currentPositionLeft + PositionsOuttake.jointRotation90 * rotLevel);
        servoRight.setPosition(currentPositionRight + PositionsOuttake.jointRotation90 * rotLevel);
    }

    public void rotateRight() {
        rotLevel--;
        rotLevel = (rotLevel + 3) % 4;

        servoLeft.setPosition(currentPositionLeft + PositionsOuttake.jointRotation90 * rotLevel);
        servoRight.setPosition(currentPositionRight + PositionsOuttake.jointRotation90 * rotLevel);
    }
}
