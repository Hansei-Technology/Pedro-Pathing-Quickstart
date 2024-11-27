package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.htech.classes.PIDController;
import org.firstinspires.ftc.teamcode.htech.config.Motors;
import org.firstinspires.ftc.teamcode.htech.config.PositionsExtendo;

public class ExtendoSystem {
    private DcMotorEx motor;
    public int currentPos = 0;
    public int target_position = 0;
    public PIDController pidController;

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
        target_position = PositionsExtendo.ground;
        pidController.targetValue = target_position;
    }

    public void goToMax() {
        target_position = PositionsExtendo.max;
        pidController.targetValue = target_position;
    }

    public void goToMid() {
        target_position = PositionsExtendo.mid;
        pidController.targetValue = target_position;
    }

    public void moveFree(double power) {
//        if(Math.abs(power) > 0.05) {
            target_position = currentPos + (int)(power * PositionsExtendo.freeSpeed);
            pidController.targetValue = target_position;
        //}
    }

    public boolean isDown() {
        return currentPos < PositionsExtendo.ground + 15 && target_position == PositionsExtendo.ground;
    }

    public void update() {
        currentPos = motor.getCurrentPosition();
        double power = pidController.update(currentPos);
        setPower(power);
    }
}
