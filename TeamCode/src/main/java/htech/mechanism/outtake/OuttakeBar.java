//package htech.mechanism.outtake;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import htech.config.PositionsOuttake;
//import htech.config.Servos;
//
//public class OuttakeBar {
//    private final Servo servoLeft;
//    private final Servo servoRight;
//
//    private double currentPositionLeft;
//    private double currentPositionRight;
//
//
//
//    public OuttakeBar(HardwareMap hardwareMap) {
//        servoLeft = hardwareMap.get(Servo.class, Servos.outtakeBarLeft);
//        servoRight = hardwareMap.get(Servo.class, Servos.outtakeBarRight);
//
//        servoLeft.setPosition(PositionsOuttake.specimen);
//        servoRight.setPosition(PositionsOuttake.specimen);
//
//        currentPositionLeft = PositionsOuttake.specimen;
//        currentPositionRight = PositionsOuttake.specimen;
//    }
//
//    public void goToTransfer() {
//        servoLeft.setPosition(PositionsOuttake.transfer);
//        servoRight.setPosition(PositionsOuttake.transfer);
//
//        currentPositionLeft = PositionsOuttake.transfer;
//        currentPositionRight = PositionsOuttake.transfer;
//    }
//
//    public void goToSpecimen() {
//        servoLeft.setPosition(PositionsOuttake.specimen);
//        servoRight.setPosition(PositionsOuttake.specimen);
//
//        currentPositionLeft = PositionsOuttake.specimen;
//        currentPositionRight = PositionsOuttake.specimen;
//    }
//
//    public void goToScore() {
//        servoLeft.setPosition(PositionsOuttake.score);
//        servoRight.setPosition(PositionsOuttake.score);
//
//        currentPositionLeft = PositionsOuttake.score;
//        currentPositionRight = PositionsOuttake.score;
//    }
//}
