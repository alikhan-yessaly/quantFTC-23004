package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.Extender;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "Extender PID Dashboard", group = "Test")
@Config
public class ExtenderTestDashboard extends OpMode {
    private Extender extender;
    private FtcDashboard dashboard;

    // Tunable PID values in FTC Dashboard
    public static double kP = 0.0005;  // Proportional gain
    public static double kI = 0.0003; // Integral gain
    public static double kD = 0.00001; //

    public static int targetPos = 1000;

    boolean wasAPressed = false;
    boolean wasBPressed = false;

    @Override
    public void init() {
        extender = new Extender(hardwareMap, "extendB");
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        // Update PID values from dashboard
        extender.setPID(kP, kI, kD);

        // Control arm position with gamepad
        if (gamepad1.a && !wasAPressed) {
            extender.setPosition(targetPos);// Move to target position
        }
        else if (gamepad1.b && !wasBPressed) {
            extender.setPosition(0); // Reset position
        }

        extender.update();

        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;

        // Send telemetry data to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Position", targetPos);
        packet.put("Current Position", extender.getCurrentPosition());
        packet.put("At Target?", extender.isAtTarget());
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        dashboard.sendTelemetryPacket(packet);

        // Also send to driver station telemetry
        telemetry.addData("Target Position", targetPos);
        telemetry.addData("Current Position", extender.getCurrentPosition());
        telemetry.addData("At Target?", extender.isAtTarget());
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}

