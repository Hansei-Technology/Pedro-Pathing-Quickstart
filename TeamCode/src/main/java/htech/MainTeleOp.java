package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.classes.StickyGamepad;
import htech.subsystem.ChassisMovement;
import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;

@TeleOp(name = "[TELEOP] Main", group = "HTech")
public class MainTeleOp extends LinearOpMode {
    ChassisMovement chassisMovement;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    //HangSystem hang;
    ElapsedTime timer;
    ElapsedTime matchTimer;
    RobotSystems robotSystems;
//    ChassisFollower chassisFollower;

    @Override
    public void runOpMode() throws InterruptedException {
        // SUBSYSTEMS //
        chassisMovement = new ChassisMovement(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        chassisFollower = new ChassisFollower(hardwareMap);

        // CLASSES //
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2, this);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1, this);

        waitForStart();

        intakeSubsystem.init();
        outtakeSubsystem.init();


        matchTimer.reset();

        //lift.goToMinusPark();
        //lift.reset();
        while (opModeIsActive()) {

            chassisMovement.move(gamepad1);
//            chassisFollower.move(gamepad1);

            //intake
            if (stickyGamepad2.a) {
                lift.goToHighChamber();
                outtakeSubsystem.goToSpecimenScore();
            }

            if(stickyGamepad2.right_stick_button){
                intakeSubsystem.hopPeSpateCollect();
            }

            if(stickyGamepad2.b) {
                intakeSubsystem.collect(false);
            }
            if(stickyGamepad2.dpad_right) {
                intakeSubsystem.collectFast();
            }
            if (gamepad2.y) {
                intakeSubsystem.goToWall();
            }
            //transfer
            if(gamepad2.x) {
                robotSystems.startTransfer();
            }

            if (stickyGamepad2.right_bumper) intakeSubsystem.claw.toggle();
            if (stickyGamepad2.left_bumper) intakeSubsystem.rotation.togglePerpendicular();


            if(gamepad2.dpad_down)  {
                extendo.goToGround();
                intakeSubsystem.goToWall();
            }
            if(gamepad2.dpad_up) {
                extendo.goToPos(230);
                intakeSubsystem.goDown();
            }

            extendo.moveFree(gamepad2.right_trigger - gamepad2.left_trigger);


            //lift control
//            if(gamepad1.b) {
//                lift.goToHighChamber();
//                outtakeSubsystem.goToSpecimenScore();
//            }
            if(gamepad1.y) {
                lift.goToHighBasket();
                outtakeSubsystem.goToSampleScore();
            }
            if(gamepad1.a) {
                lift.goToGround();
                outtakeSubsystem.goToTransfer();
                if(intakeSubsystem.intakeState == IntakeSubsystem.IntakeState.TRANSFER) intakeSubsystem.goToReady();
            }

            if(gamepad1.right_bumper) outtakeSubsystem.claw.open();
            if(gamepad1.left_bumper) robotSystems.scoreSpecimen();


            if(matchTimer.seconds() > 90) { //only in endgame
                //hang.setPower(-gamepad2.right_stick_y);
            }

            if(stickyGamepad2.dpad_left) {
                robotSystems.outtakeSubsystem.joint.rotateRight();
            }
            if(stickyGamepad2.dpad_right) {
                robotSystems.outtakeSubsystem.joint.rotateLeft();
            }


            stickyGamepad2.update();
            stickyGamepad1.update();
            robotSystems.update();

            if(gamepad1.left_stick_button) {
                while (gamepad1.left_stick_button) {
                    lift.setPower(-0.35);
                }
                lift.setPower(0);
                lift.reset(gamepad1);
                gamepad1.rumble(100);
            }

            //telemetry:
            telemetry.addData("[STATUS]", "Main Teleop is running.");
            telemetry.addData("Match Time", matchTimer.seconds());
            telemetry.addData("Lift", lift.currentPos);
            telemetry.addData("Extendo", extendo.currentPos);
            telemetry.addData("Intake", intakeSubsystem.intakeState);
            telemetry.addData("intakeTimer", robotSystems.timerCollect.milliseconds());
            telemetry.addData("extendoPID", extendo.pidEnabled);
            telemetry.addData("outtakeRot", robotSystems.outtakeSubsystem.joint.getRot());

            telemetry.update();
        }
    }

}
