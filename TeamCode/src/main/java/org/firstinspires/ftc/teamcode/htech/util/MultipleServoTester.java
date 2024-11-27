package org.firstinspires.ftc.teamcode.htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "[UTIL] Multiple Servo Tester", group = "HTech")
public class MultipleServoTester extends LinearOpMode {
    public static int length = 2;
    public static String[] servos = new String[10];
    public static double[] positions = new double[10];

    @Override
    public void runOpMode() throws InterruptedException {
        servos = new String[length];
        positions = new double[length];
        Arrays.fill(servos, "");
        length = 2;

        FtcDashboard.getInstance().updateConfig();
        waitForStart();

        while (opModeIsActive()) {

        }
    }

    private void resetData() {
        if (servos != null && positions != null) {
            String[] tempServos = servos.clone();
            double[] tempPositions = positions.clone();

            servos = new String[length];
            positions = new double[length];
            Arrays.fill(servos, "");

            for (int i = 0; i < tempServos.length && i < length; i++) {
                if (tempServos[i] != null) {
                    servos[i] = tempServos[i];
                }
                positions[i] = tempPositions[i];
            }
        } else {
            servos = new String[length];
            positions = new double[length];
            Arrays.fill(servos, "");
        }
    }
}
