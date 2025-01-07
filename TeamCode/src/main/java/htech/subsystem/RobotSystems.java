package htech.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.config.RobotSettings;

public class RobotSystems {
    public ExtendoSystem extendoSystem;
    public LiftSystem liftSystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public ElapsedTime timer;
    public ElapsedTime timerCollect;
    public ElapsedTime timerSpecimen;

    public RobotSystems(ExtendoSystem extendoSystem, LiftSystem liftSystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.extendoSystem = extendoSystem;
        this.liftSystem = liftSystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        timer = new ElapsedTime();
        timerCollect = new ElapsedTime();
        timerSpecimen = new ElapsedTime();
    }

    public void update() {
        extendoSystem.update();
        liftSystem.update();
        intakeSubsystem.update();
        updateTranfer();
        updateCollect();
        updateSpecimen();
        updateHopPeSpate();
    }

    public void startTransfer() {
        if(intakeSubsystem.claw.isOpen) {
            intakeSubsystem.claw.close();
            transferState = TransferStates.CLOSING_CLAW;
        } else {
            transferState = TransferStates.LIFT_GOING_DOWN;
        }
    }


    public void scoreSpecimen() {
        specimenState = SpecimenStates.GOING_TO_SPECIMEN;
    }

    public enum TransferStates {
        IDLE,
        CLOSING_CLAW,
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

    public enum HopPeSpateStates {
        IDLE,
        CASE1,
        CASE2,
        CASE3,
        CASE4,
        CASE5
    }
    public HopPeSpateStates hopPeSpateState = HopPeSpateStates.IDLE;

    public boolean firstTime = true;


    public enum SpecimenStates {
        IDLE,
        GOING_TO_SPECIMEN,
        WAITING_TO_CATCH,
        LIFT_GOING_DOWN
    }
    public SpecimenStates specimenState = SpecimenStates.IDLE;

    void updateCollect() {
        switch (intakeSubsystem.intakeState) {
            case COLLECT_GOING_DOWN:
                if(firstTime) {
                    timerCollect.reset();
                    firstTime = false;
                }
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingDownFast && intakeSubsystem.fastCollect) {
                    intakeSubsystem.claw.close();
                    outtakeSubsystem.joint.goToTransfer();
                    timerCollect.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                }
                if(timerCollect.milliseconds() > RobotSettings.timeToCollectGoingDown && !intakeSubsystem.claw.isOpen) {
                    //intakeSubsystem.claw.close();
                    timerCollect.reset();
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLLECTING;
                }
                break;
            case COLLECTING:
                firstTime = true;
                if(!intakeSubsystem.hopPeSpate && intakeSubsystem.fastCollect && timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    transferState = TransferStates.LIFT_GOING_DOWN;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.DOWN;
                    timerCollect.reset();
                    intakeSubsystem.fastCollect = false;
                }

                if(intakeSubsystem.hopPeSpate && intakeSubsystem.fastCollect && timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    hopPeSpateState = HopPeSpateStates.CASE1;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.DOWN;
                    timerCollect.reset();
                    intakeSubsystem.fastCollect = false;
                    intakeSubsystem.hopPeSpate = false;
                }

                if(timerCollect.milliseconds() > RobotSettings.timeToCollect) {
                    intakeSubsystem.goToWall();
                    extendoSystem.goToGround();
                    //transferState = TransferStates.LIFT_GOING_DOWN;
                    intakeSubsystem.intakeState = IntakeSubsystem.IntakeState.COLECT_GOING_UP;
                }
                break;
        }
    }


    void updateTranfer() {
        switch (transferState) {
            case IDLE:
                break;

            case CLOSING_CLAW:
                if(timer.milliseconds() > RobotSettings.timeToCloseClaw) {
                    timer.reset();
                    transferState = TransferStates.LIFT_GOING_DOWN;
                }
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
                if ((liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeDown_Transfer) || timer.milliseconds() > RobotSettings.timeFailedToCloseLift) {
                    intakeSubsystem.goToTransfer();
                    timer.reset();
                    transferState = TransferStates.READY_TO_TRANSFER;
                }

                break;

            case INTAKE_WALL:
                if ((liftSystem.isDown() && extendoSystem.isDown() && timer.milliseconds() > RobotSettings.timeWall_Transfer) || timer.milliseconds() > RobotSettings.timeFailedToCloseLift) {
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
                if(timer.milliseconds() > RobotSettings.timeToCatch /* && intakeSubsystem.isAtPos()*/) {
                    timer.reset();
                    intakeSubsystem.claw.open();
                    transferState = TransferStates.WAITING_TO_CATCH;
                }
                break;
            case WAITING_TO_CATCH:
                if (timer.milliseconds() > RobotSettings.timeWaitingToCatch) {
                    timer.reset();
                    intakeSubsystem.goToWall();
                    outtakeSubsystem.goToSpecimenScore();
                    transferState = TransferStates.TRANSFER_READY;
                }
                break;
            case TRANSFER_READY:
                if (timer.milliseconds() > RobotSettings.timeToCloseOuttake) {
                    timer.reset();
                    transferState = TransferStates.IDLE;
                }
                break;
        }
    }

    public void updateSpecimen() {
        switch (specimenState) {
            case IDLE:
                break;
            case GOING_TO_SPECIMEN:
                liftSystem.goToMagicPos();
                if(liftSystem.isAtPosition()) {
                    timerSpecimen.reset();
                    outtakeSubsystem.claw.open();
                    specimenState = SpecimenStates.WAITING_TO_CATCH;
                }

                break;
            case WAITING_TO_CATCH:
                if(timerSpecimen.milliseconds() > RobotSettings.time_to_specimen) {
                    liftSystem.goToGround();
                    outtakeSubsystem.goToTransfer();
                    specimenState = SpecimenStates.LIFT_GOING_DOWN;
                }
                break;
            case LIFT_GOING_DOWN:
                if(liftSystem.isDown()) {
                    specimenState = SpecimenStates.IDLE;
                }
                break;
        }
    }

    public void updateHopPeSpate(){
        switch (hopPeSpateState) {
            case IDLE:
                break;

            case CASE1:
                outtakeSubsystem.joint.catapultarePos();
                extendoSystem.goToGround();
                intakeSubsystem.goToReady();
                hopPeSpateState = HopPeSpateStates.CASE2;
                timer.reset();
                break;

            case CASE2:
                if(timer.milliseconds() > RobotSettings.timeToHopPeSpate) {
                    intakeSubsystem.claw.open();
                    hopPeSpateState = HopPeSpateStates.IDLE;
                }
                break;
        }

    }
}
