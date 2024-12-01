package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.htech.classes.PIDController;
import org.firstinspires.ftc.teamcode.htech.config.Motors;
import org.firstinspires.ftc.teamcode.htech.config.PositionsLift;

//2 motor lift system with PID
public class LiftSystem {
    private DcMotorEx left, right;
    int target_position = 0;
    public PIDController pidController;
    public int currentPos = 0;

    public LiftSystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Motors.lift2);
        right = hardwareMap.get(DcMotorEx.class, Motors.lift1);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.FORWARD);

        pidController = new PIDController(PositionsLift.kP, PositionsLift.kI, PositionsLift.kD);
        pidController.targetValue = target_position;
        pidController.maxOutput = 1;
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void goToGround() {
        target_position = PositionsLift.ground;
        pidController.targetValue = target_position;
    }

    public void goToHighChamber() {
        target_position = PositionsLift.highChamber;
        pidController.targetValue = target_position;
    }

    public void goToLowChamber() {
        target_position = PositionsLift.lowChamber;
        pidController.targetValue = target_position;
    }

    public void goToLowBasket() {
        target_position = PositionsLift.lowBasket;
        pidController.targetValue = target_position;
    }

    public void goToHighBasket() {
        target_position = PositionsLift.highBasket;
        pidController.targetValue = target_position;
    }

    public void goToMagicPos() {
        target_position = PositionsLift.magic;
        pidController.targetValue = target_position;
    }

    public boolean isDown() {
        return currentPos < PositionsLift.ground + 15 && target_position == PositionsLift.ground;
    }

    public void goToPark() {
        target_position = PositionsLift.park;
        pidController.targetValue = target_position;
    }



    public boolean isAtPosition() {
        return Math.abs(currentPos - target_position) < 10;
    }

    public void update() {
        currentPos = left.getCurrentPosition();
        double power = pidController.update(currentPos);
        setPower(power);
    }
}
