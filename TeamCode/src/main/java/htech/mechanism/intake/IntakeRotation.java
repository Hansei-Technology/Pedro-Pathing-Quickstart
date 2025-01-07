package htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import htech.config.PositionsIntake;
import htech.config.RobotSettings;
import htech.config.Servos;

public class IntakeRotation {
    private final Servo rotationServo;
    private double currentPosition;
    public double rotLevel = 0;

    public IntakeRotation(HardwareMap hardwareMap) {
        rotationServo = hardwareMap.get(Servo.class, Servos.intakeRotationServo);
//        goToNormal();
    }

    public void goToNormal() {
        rotationServo.setPosition(PositionsIntake.normalRotation);
        currentPosition = PositionsIntake.normalRotation;
    }

    public void goToPerpendicular() {
        rotationServo.setPosition(PositionsIntake.perpendicularRotation);
        currentPosition = PositionsIntake.perpendicularRotation;
    }

    public void goToFlipped() {
        rotationServo.setPosition(PositionsIntake.flippedNormalRotation);
        currentPosition = PositionsIntake.flippedNormalRotation;
    }

    public void flip() {
        if (currentPosition == PositionsIntake.normalRotation) {
            this.goToFlipped();
        } else {
            this.goToNormal();
        }
    }

    public void togglePerpendicular() {
        if (currentPosition == PositionsIntake.flippedNormalRotation) {
            this.goToPerpendicular();
        } else {
            this.goToFlipped();
        }
    }

    public boolean isAtPos(){
        return Math.abs(rotationServo.getPosition() - currentPosition) < 0.01;
    }

    public void goToPos(double pos){
        rotationServo.setPosition(pos);
        currentPosition = pos;
    }

    public void goToAutoPos(){
        rotationServo.setPosition(PositionsIntake.rotationAuto);
        currentPosition = PositionsIntake.rotationAuto;
    }


    public void handleRotation(double pow) {
        rotLevel += pow * PositionsIntake.rotSpeed;

        if(rotLevel > 1) {
            rotLevel = 1;
        } else if(rotLevel < -6) {
            rotLevel = -6;
        }

        rotationServo.setPosition(PositionsIntake.flippedNormalRotation + rotLevel * PositionsIntake.rotation30Deg);
        currentPosition = PositionsIntake.flippedNormalRotation + rotLevel * PositionsIntake.rotation30Deg;
    }

    public void update() {
        rotationServo.setPosition(currentPosition);
    }


}