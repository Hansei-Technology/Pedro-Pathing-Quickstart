package htech.subsystem;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.classes.PIDController;
import htech.config.Motors;
import htech.config.PositionsLift;

//2 motor lift system with PID
public class LiftSystem {
    private final DcMotorEx left, right;
//    private DcMotorEx encoder;
    public int target_position = 0;
    public PIDController pidController;
    public int currentPos = 0;
    public boolean PIDON = true;

    public LiftSystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Motors.lift2);
        right = hardwareMap.get(DcMotorEx.class, Motors.lift1);
//        encoder = hardwareMap.get(DcMotorEx.class, Motors.liftEncoder);

//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

//        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        right.setDirection(DcMotorEx.Direction.REVERSE);
        left.setDirection(DcMotorEx.Direction.FORWARD);

        pidController = new PIDController(PositionsLift.kP, PositionsLift.kI, PositionsLift.kD);
        pidController.targetValue = target_position;
        pidController.maxOutput = 1;
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void goToGround() {
        PIDON = true;
        target_position = PositionsLift.ground;
        pidController.targetValue = target_position;
    }

    public void goToHighChamber() {
        PIDON = true;
        target_position = PositionsLift.highChamber;
        pidController.targetValue = target_position;
    }

    public void goToLowChamber() {
        PIDON = true;
        target_position = PositionsLift.lowChamber;
        pidController.targetValue = target_position;
    }

    public void goToLowBasket() {
        PIDON = true;
        target_position = PositionsLift.lowBasket;
        pidController.targetValue = target_position;
    }

    public void goToHighBasket() {
        PIDON = true;
        target_position = PositionsLift.highBasket;
        pidController.targetValue = target_position;
    }

    public void goToMagicPos() {
        PIDON = true;
        target_position = PositionsLift.magic;
        pidController.targetValue = target_position;
    }

    public boolean isDown() {
        return currentPos < PositionsLift.ground + 25 && target_position == PositionsLift.ground;
    }

    public void goToPark() {
        target_position = PositionsLift.park;
        pidController.targetValue = target_position;
    }

    public void goToMinusPark() {
        target_position = -PositionsLift.park;
        pidController.targetValue = target_position;
    }

    public void reset(Gamepad g) {
        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentPos = 0;
        target_position = 0;
        g.rumble(100);
        PIDON = true;
    }




    public boolean isAtPosition() {
        return Math.abs(currentPos - target_position) < 10;
    }

    public void update() {
        if(PIDON) {
            currentPos = left.getCurrentPosition();
            double power = pidController.update(currentPos);
            setPower(power);
        }
    }
}
