package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeBar;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeJoint;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeRotation;

@Config
public class IntakeSubsystem {
    public final IntakeClaw claw;
    public final IntakeRotation rotation;
    public final IntakeBar bar;
    public final IntakeJoint joint;
    public final ElapsedTime timer;

    public enum IntakeState {
        DOWN,
        WALL,
        READY,
        TRANSFER,
        COLLECT_GOING_DOWN,
        COLLECTING,
        COLECT_GOING_UP
    }
    public IntakeState CS = IntakeState.DOWN;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        // MECHANISM //
        claw = new IntakeClaw(hardwareMap);
        rotation = new IntakeRotation(hardwareMap);
        bar = new IntakeBar(hardwareMap);
        joint = new IntakeJoint(hardwareMap);
        timer = new ElapsedTime();
    }

    public void goDown() {
        joint.goToPickup();
        bar.goToGround();
        rotation.goToFlipped();
        if(CS != IntakeState.COLLECTING) CS = IntakeState.DOWN;
    }

    public void goToWall() {
        joint.goToWall();
        bar.goToWall();
        rotation.goToFlipped();
        CS = IntakeState.WALL;
    }

    public void goToReady() {
        joint.goToPreTransfer();
        bar.goToTransfer();
        rotation.goToNormal();
        CS = IntakeState.READY;
    }

    public void goToTransfer() {
        joint.goToTransfer();
        bar.goToTransfer();
        CS = IntakeState.TRANSFER;
    }

    public void collect() { //if the intake is already down collect a pixel, if not go down
        if(CS == IntakeState.DOWN) {
            joint.goToCollect();
            bar.goToCollect();
            CS = IntakeState.COLLECT_GOING_DOWN;
            timer.reset();
        }
        else goDown();
    }

    public void updateColect() {
        switch (CS) {
            case COLLECT_GOING_DOWN:
                if(timer.milliseconds() > RobotSettings.timeToCollectGoingDown) {
                    claw.close();
                    timer.reset();
                    CS = IntakeState.COLLECTING;
                }
                break;
            case COLLECTING:
                if(timer.milliseconds() > RobotSettings.timeToCollect) {
                    goDown();
                    CS = IntakeState.COLECT_GOING_UP;
                }
                break;
            case COLECT_GOING_UP:
                if(timer.milliseconds() > RobotSettings.timeToCollectGoingUp) {
                    CS = IntakeState.DOWN;
                }
                break;
        }
    }
}
