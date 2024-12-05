package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.htech.classes.PIDController;
import org.firstinspires.ftc.teamcode.htech.config.Motors;
import org.firstinspires.ftc.teamcode.htech.config.PositionsExtendo;
import org.firstinspires.ftc.teamcode.htech.config.PositionsLift;

public class ExtendoSystem {
    private DcMotorEx motor;
    public int currentPos = 0;
    public int target_position = 0;
    public PIDController pidController;
    public boolean pidEnabled = true;
    public boolean last = false;

    public ExtendoSystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Motors.extendo);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        pidController = new PIDController(PositionsExtendo.kP, PositionsExtendo.kI, PositionsExtendo.kD);

        pidController.targetValue = target_position;
        pidController.maxOutput = 1;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void goToGround() {
        pidEnabled = true;
        target_position = PositionsExtendo.ground;
        pidController.targetValue = target_position;
    }

    public void goToMax() {
        pidEnabled = true;
        target_position = PositionsExtendo.max;
        pidController.targetValue = target_position;
    }

    public void goToMid() {
        pidEnabled = true;
        target_position = PositionsExtendo.mid;
        pidController.targetValue = target_position;
    }

    public void moveFree(double power) {
        if(Math.abs(power) > 0.2) {
            pidEnabled = false;
            if(currentPos >= PositionsExtendo.max && power > 0) {
                power = 0;
            }
            if (currentPos <= PositionsExtendo.ground && power < 0) {
                power = 0;
            }
            motor.setPower(Math.signum(power) * PositionsExtendo.freeSpeed);
            last = true;
        } else if(last) {
            last = false;
            motor.setPower(0);
        }
    }

    public boolean isDown() {
        return currentPos < PositionsExtendo.ground + 15 && target_position == PositionsExtendo.ground;
    }

    public void goToPos(int poz) {
        pidEnabled = true;
        target_position = poz;
        pidController.targetValue = target_position;
    }

    public void update() {
        currentPos = motor.getCurrentPosition();
        if(pidEnabled) {
            double power = pidController.update(currentPos);
            setPower(power);
        }

    }
}
