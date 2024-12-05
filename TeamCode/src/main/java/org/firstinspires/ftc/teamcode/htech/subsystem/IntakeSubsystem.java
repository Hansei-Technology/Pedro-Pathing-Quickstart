package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeBarMotionProfile;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeJoint;
import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeRotation;

@Config
public class IntakeSubsystem {
    public final IntakeClaw claw;
    public final IntakeRotation rotation;
    public final IntakeBarMotionProfile bar;
    public final IntakeJoint joint;

    boolean fastCollect = false;

    public enum IntakeState {
        DOWN,
        WALL,
        READY,
        TRANSFER,
        COLLECT_GOING_DOWN,
        COLLECTING,
        COLECT_GOING_UP
    }
    public IntakeState intakeState = IntakeState.DOWN;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        // MECHANISM //
        claw = new IntakeClaw(hardwareMap);
        rotation = new IntakeRotation(hardwareMap);
        bar = new IntakeBarMotionProfile(hardwareMap);
        joint = new IntakeJoint(hardwareMap);
    }

    public void goDown() {
        joint.goToPickup();
        bar.goToGround();
        rotation.goToFlipped();
        if(intakeState != intakeState.COLLECTING) intakeState = intakeState.DOWN;
    }

    public void goToWall() {
        joint.goToWall();
        bar.goToWall();
        rotation.goToFlipped();
        intakeState = intakeState.WALL;
    }

    public void goToReady() {
        joint.goToPreTransfer();
        bar.goToTransfer();
        rotation.goToNormal();
        intakeState = intakeState.READY;
    }

    public void goToTransfer() {
        joint.goToTransfer();
        bar.goToTransfer();
        intakeState = intakeState.TRANSFER;
    }

    public void collect() { //if the intake is already down collect a pixel, if not go down
        if(intakeState == intakeState.DOWN) {
            joint.goToCollect();
            bar.goToCollect();
            intakeState = intakeState.COLLECT_GOING_DOWN;
        }
        else goDown();
    }

    public void collectFast() {
        fastCollect = true;
        collect();
    }

    public void update() {
        bar.update();
    }
}
