package org.firstinspires.ftc.teamcode.htech.mechanism.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.PositionsIntake;
import org.firstinspires.ftc.teamcode.htech.config.PositionsOuttake;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class OuttakeClaw {
    private final Servo clawServo;
    private boolean isOpen = false;

    public OuttakeClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, Servos.outtakeClaw);
//        clawServo.setPosition(PositionsOuttake.openedClaw);
    }

    public void open() {
        clawServo.setPosition(PositionsOuttake.openedClaw);
        isOpen = true;
    }

    public void close() {
        clawServo.setPosition(PositionsOuttake.closedClaw);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            clawServo.setPosition(PositionsOuttake.closedClaw);
        } else {
            clawServo.setPosition(PositionsOuttake.openedClaw);
        }

        isOpen = !isOpen;
    }
}
