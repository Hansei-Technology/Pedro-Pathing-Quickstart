package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    ElapsedTime timer;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.updateColect();
    }


    enum TransferStates {
        IDLE,
        LIFT_GOING_DOWN,
        INTAKE_DOWN,
        INTAKE_WALL,
        READY_TO_TRANSFER,
        WAITING_TO_FALL,
        TRANSFER_READY,
        LIFT_GOING_UP,
    }

    public TransferStates transferState = TransferStates.IDLE;

    void updateTranfer() {
        switch (transferState) {
            case IDLE:
                break;

            case LIFT_GOING_DOWN:
                //on entry
                liftSystem.goToGround();
                extendoSystem.goToGround();
                outtakeSubsystem.goToTransfer();
                outtakeSubsystem.claw.open();
                timer.reset();

                //condition to exit
                if (intakeSubsystem.CS == IntakeSubsystem.IntakeState.DOWN) {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_DOWN;
                } else {
                    intakeSubsystem.goToReady();
                    transferState = TransferStates.INTAKE_WALL;
                }
                break;

            case INTAKE_DOWN:
                if (liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeDown_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case INTAKE_WALL:
                if (liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeWall_Transfer) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case READY_TO_TRANSFER:
                if (timer.milliseconds() > RobotSettings.timeReady_Transfer) {
                    timer.reset();
                    intakeSubsystem.claw.open();
                    intakeSubsystem.claw.open();
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.WAITING_TO_FALL;
                }
                break;

            case WAITING_TO_FALL:
                if (timer.milliseconds() > RobotSettings.timeToDropElement) {
                    timer.reset();
                    intakeSubsystem.goToWall();
                    transferState = TransferStates.TRANSFER_READY;
                }
                break;
            case TRANSFER_READY:
                if (timer.milliseconds() > RobotSettings.timeToCloseOuttake) {
                    timer.reset();

                    transferState = TransferStates.IDLE;
                }
        }
    }
}
