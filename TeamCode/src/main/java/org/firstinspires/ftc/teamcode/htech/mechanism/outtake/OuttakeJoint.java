package org.firstinspires.ftc.teamcode.htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.PositionsOuttake;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class OuttakeJoint {
    private final Servo servoLeft;
    private final Servo servoRight;

    private double currentPositionLeft;
    private double currentPositionRight;

    public OuttakeJoint(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, Servos.outtakeLeft);
        servoRight = hardwareMap.get(Servo.class, Servos.outtakeRight);

        servoLeft.setPosition(PositionsOuttake.jointTransferLeft);
        servoRight.setPosition(PositionsOuttake.jointTransferRight);

        currentPositionLeft = PositionsOuttake.jointTransferLeft;
        currentPositionRight = PositionsOuttake.jointTransferRight;
    }

    public void goToTransfer() {
        servoLeft.setPosition(PositionsOuttake.jointTransferLeft);
        servoRight.setPosition(PositionsOuttake.jointTransferRight);

        currentPositionLeft = PositionsOuttake.jointTransferLeft;
        currentPositionRight = PositionsOuttake.jointTransferRight;
    }

    public void goToSpecimenScore() {
        servoLeft.setPosition(PositionsOuttake.jointSpecimenLeft);
        servoRight.setPosition(PositionsOuttake.jointSpecimenRight);

        currentPositionLeft = PositionsOuttake.jointSpecimenLeft;
        currentPositionRight = PositionsOuttake.jointSpecimenRight;
    }

    public void goToBasketScore() {
        servoLeft.setPosition(PositionsOuttake.jointBasketLeft);
        servoRight.setPosition(PositionsOuttake.jointBasketRight);

        currentPositionLeft = PositionsOuttake.jointBasketLeft;
        currentPositionRight = PositionsOuttake.jointBasketRight;
    }
}
