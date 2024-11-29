package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.htech.mechanism.intake.IntakeBar;
import org.firstinspires.ftc.teamcode.htech.mechanism.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.htech.mechanism.outtake.OuttakeJoint;
@Config
public class OuttakeSubsystem {
    public final OuttakeClaw claw;
    public final OuttakeJoint joint;

    public enum outtakeStates {
        SPECIMEN,
        BASKET,
        TRANSFER
    }
    public outtakeStates CS = outtakeStates.TRANSFER;
    //timers are in milliseconds
    public static int timeToMove = 100;


    public OuttakeSubsystem(HardwareMap hardwareMap) {
        // MECHANISM //
        claw = new OuttakeClaw(hardwareMap);
        joint = new OuttakeJoint(hardwareMap);
    }

    public void goToTransfer() {
        joint.goToTransfer();
        claw.open();
    }

    public void goToSampleScore() {
        joint.goToBasketScore();
    }

    public void goToSpecimenScore() {
        joint.goToSpecimenScore();
    }
}
