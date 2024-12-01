package org.firstinspires.ftc.teamcode.htech.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.classes.HanseiTelemetry;

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
    private HanseiTelemetry htelemetry;

    public static boolean configure = true;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        htelemetry = new HanseiTelemetry(mtelemetry);
        htelemetry.clear();

        if (length <= 0 || length > 10) {
            htelemetry.error("Length must be between 1 and 10.");
            htelemetry.info("This tester will now stop.");
            htelemetry.update();
            stop();
            return;
        }

        if (configure) {
            htelemetry.status("CONFIG");
            htelemetry.info("Configuration mode will change the length of the arrays, while keeping data.");
            htelemetry.info("After this, it will stop, and you'll be able to freely configure the tester.");
            htelemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                htelemetry.clear();
                return;
            }

            resetData();
            htelemetry.success("Data has been updated successfully!");
            htelemetry.update();
            stop();
            return;
        }

        htelemetry.status("INIT");
        htelemetry.info("All checks passed. Start the OpMode to register servos.");
        htelemetry.update();

        waitForStart();
        Servo[] servoInstances = new Servo[servos.length];

        for (int i = 0; i < servos.length; i++) {
            htelemetry.status("INIT");
            htelemetry.status("Initializing Servos...");

            try {
                servoInstances[i] = hardwareMap.get(Servo.class, servos[i]);

                for (int j = 0; j <= i; j++) {
                    htelemetry.success("Initialized servo '" + servos[j] + "'.");
                }
                htelemetry.update();
            } catch(Exception e) {
                htelemetry.error("Servo with value '" + servos[i] + "' does not exist!");
                htelemetry.error("Program will now exit");
                htelemetry.update();
                stop();
                return;
            }
        }

        htelemetry.clear();

        while (opModeIsActive()) {
            htelemetry.status("RUNNING");
            htelemetry.update();
        }
    }

    /**
     * Resets the servo and position arrays. Not only that, this also keeps the previous data.
     */
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

        FtcDashboard.getInstance().updateConfig();
    }
}
