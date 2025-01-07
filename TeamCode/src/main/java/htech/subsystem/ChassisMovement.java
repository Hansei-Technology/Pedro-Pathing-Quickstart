package htech.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.config.Motors;
import htech.config.RobotSettings;

public class ChassisMovement {
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx rightRearMotor;

//    private final Follower follower;

    private static final double speed = RobotSettings.speed;
    private static final double rotationSpeed = RobotSettings.rotationSpeed;

    public ChassisMovement(HardwareMap hardwareMap) {
        // MOTOR DECLARATION //
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.leftFrontMotor);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.rightFrontMotor);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, Motors.leftRearMotor);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, Motors.rightRearMotor);

//        follower = new Follower(hardwareMap);

        // MOTOR CONFIGURATION //
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        follower.startTeleopDrive();
    }

    public void move(Gamepad g) {
        if(g.right_trigger > 0.05 && g.left_trigger > 0.05) {
            updateMovementSlowRotationReverse(g);
        } else if(g.right_trigger > 0.05) {
            updateMovementReverse(g);
        } else if (g.left_trigger > 0.05) {
            updateMovementSlowRotation(g);
        } else {
            updateMovement(g);
        }
    }

    public void updateMovement(Gamepad g) {
        leftFrontMotor.setPower((-g.left_stick_y + g.left_stick_x + g.right_stick_x * rotationSpeed) * speed);
        rightFrontMotor.setPower((-g.left_stick_y - g.left_stick_x - g.right_stick_x * rotationSpeed) * speed);
        leftRearMotor.setPower((-g.left_stick_y - g.left_stick_x + g.right_stick_x * rotationSpeed) * speed);
        rightRearMotor.setPower((-g.left_stick_y + g.left_stick_x - g.right_stick_x * rotationSpeed) * speed);
//        follower.setTeleOpMovementVectors(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x);
//        follower.update();
    }

    public void updateMovementReverse(Gamepad g){
        leftFrontMotor.setPower((g.left_stick_y - g.left_stick_x + g.right_stick_x * rotationSpeed) * speed);
        rightFrontMotor.setPower((g.left_stick_y + g.left_stick_x - g.right_stick_x * rotationSpeed) * speed);
        leftRearMotor.setPower((g.left_stick_y + g.left_stick_x + g.right_stick_x * rotationSpeed) * speed);
        rightRearMotor.setPower((g.left_stick_y - g.left_stick_x - g.right_stick_x * rotationSpeed) * speed);
//        follower.setTeleOpMovementVectors(g.left_stick_y, g.left_stick_x, g.right_stick_x);
//        follower.update();
    }

    public void updateMovementSlowRotation(Gamepad g){
        leftFrontMotor.setPower((-g.left_stick_y + g.left_stick_x + g.right_stick_x * 0.3) * speed);
        rightFrontMotor.setPower((-g.left_stick_y - g.left_stick_x - g.right_stick_x * 0.3) * speed);
        leftRearMotor.setPower((-g.left_stick_y - g.left_stick_x + g.right_stick_x * 0.3) * speed);
        rightRearMotor.setPower((-g.left_stick_y + g.left_stick_x - g.right_stick_x * 0.3) * speed);
//        follower.setTeleOpMovementVectors(-g.left_stick_y, -g.left_stick_x, -g.right_stick_x);
//        follower.update();
    }

    public void updateMovementSlowRotationReverse(Gamepad g){
        leftFrontMotor.setPower((g.left_stick_y - g.left_stick_x + g.right_stick_x * 0.3) * speed);
        rightFrontMotor.setPower((g.left_stick_y + g.left_stick_x - g.right_stick_x * 0.3) * speed);
        leftRearMotor.setPower((g.left_stick_y + g.left_stick_x + g.right_stick_x * 0.3) * speed);
        rightRearMotor.setPower((g.left_stick_y - g.left_stick_x - g.right_stick_x * 0.3) * speed);
//        follower.setTeleOpMovementVectors(g.left_stick_y, g.left_stick_x, g.right_stick_x);
//        follower.update();
    }
}
