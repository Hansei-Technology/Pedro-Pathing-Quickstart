package org.firstinspires.ftc.teamcode.htech.mechanism.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;
import org.firstinspires.ftc.teamcode.htech.config.Servos;

public class IntakeRotitor {
    private CRServo servo;
    public boolean isOn = false;
    public boolean isSpitting = false;
    ElapsedTime timer;

    public IntakeRotitor(HardwareMap hardwareMap) {
        servo = hardwareMap.crservo.get(Servos.intakeClawServo);
        timer = new ElapsedTime();
    }

    public void handle(Gamepad g) {
        if(g.right_bumper) {
            servo.setPower(-1);
            isOn = true;
        } else if(g.dpad_left || isSpitting) {
            servo.setPower(1);
            isOn = true;
        } else {
            servo.setPower(0);
            isOn = false;
        }
    }

    public void update() {
        if(isSpitting) {
            if(timer.milliseconds() > RobotSettings.time_to_spit) {
                isSpitting = false;
                timer.reset();
                servo.setPower(0);
            }
        }
    }

    public void spit() {
        isSpitting = true;
        timer.reset();
        servo.setPower(1);
    }

    public void turnOn() {
        
    }


}
