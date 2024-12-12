package org.firstinspires.ftc.teamcode.htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.PositionsIntake;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class IntakeJoint {
    private final Servo jointServo;
    private double currentPosition;

    public IntakeJoint(HardwareMap hardwareMap) {
        jointServo = hardwareMap.get(Servo.class, Servos.intakeJointServo);
//        jointServo.setPosition(PositionsIntake.prepTransferPositionJoint);
//        currentPosition = PositionsIntake.prepTransferPositionJoint;
    }

    public void goToPickup() {
        jointServo.setPosition(PositionsIntake.groundPositionJoint);
        currentPosition = PositionsIntake.groundPositionJoint;
    }

    public void goToWall() {
        jointServo.setPosition(PositionsIntake.wallPickupPositionJoint);
        currentPosition = PositionsIntake.wallPickupPositionJoint;
    }
    public void goToPreTransfer() {
        jointServo.setPosition(PositionsIntake.prepTransferPositionJoint);
        currentPosition = PositionsIntake.prepTransferPositionJoint;
    }

    public void goToTransfer() {
        jointServo.setPosition(PositionsIntake.finalTransferPositionJoint);
        currentPosition = PositionsIntake.finalTransferPositionJoint;
    }

    public void goToCollect() {
        jointServo.setPosition(PositionsIntake.collectPositionJoint);
        currentPosition = PositionsIntake.collectPositionJoint;
    }

}
