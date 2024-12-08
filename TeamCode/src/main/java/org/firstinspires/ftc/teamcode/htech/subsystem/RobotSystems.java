package org.firstinspires.ftc.teamcode.htech.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    ElapsedTime timer;
    public ElapsedTime timerCollect;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
        timerCollect = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.update();
        updateTranfer();
        updateCollect();
    }


    public enum TransferStates {
        IDLE,
        LIFT_GOING_DOWN,
        INTAKE_DOWN,
        INTAKE_WALL,
        READY_TO_TRANSFER,
        CATCHING,
        WAITING_TO_CATCH,
        TRANSFER_READY,
        LIFT_GOING_UP,
    }

    public TransferStates transferState = TransferStates.IDLE;


    public boolean firstTime = true;

    void updateCollect() {
        switch (intakeSubsystem.intakeState) {
            case COLLECT_GOING_DOWN:
                if(firstTime) {
                    timerCollect.reset();
                    firstTime = false;
                }
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingDown) {
                    intakeSubsystem.claw.close();
                    timerCollect.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                }
                break;
            case COLLECTING:
                firstTime = true;
                if(intakeSubsystem.fastCollect && timerCollect.milliseconds() > RobotSettings.timeToCollectFast) {
                    transferState = TransferStates.LIFT_GOING_DOWN;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.DOWN;
                    intakeSubsystem.fastCollect = false;
                }

                if(timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    intakeSubsystem.goDown();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLECT_GOING_UP;
                }
                break;
            case COLECT_GOING_UP:
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingUp) {
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.DOWN;
                }
                break;
        }
    }


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
                if (intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.DOWN) {
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
                    outtakeSubsystem.claw.close();
                    transferState = TransferStates.CATCHING;
                }
                break;
            case CATCHING:
                if(timer.milliseconds() > RobotSettings.timeToCatch) {
                    timer.reset();
                    intakeSubsystem.claw.open();
                    transferState = TransferStates.WAITING_TO_CATCH;
                }
                break;
            case WAITING_TO_CATCH:
                if (timer.milliseconds() > RobotSettings.timeWaitingToCatch) {
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
