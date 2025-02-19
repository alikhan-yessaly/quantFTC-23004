//package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.utils.Extender;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//@TeleOp(name = "Extender PID Dashboard", group = "Test")
//@Config
//public class ExtenderTestDashboard extends OpMode {
//    private Extender extender;
//    private FtcDashboard dashboard;
//
//    // Tunable PID values in FTC Dashboard
//    public static double kP = 0.002;  // Proportional gain
//    public static double kI = 0.0005; // Integral gain
//    public static double kD = 0.000005;
//
//    public static int targetPos = 1000;
//    public static int stepSize = 80; // Max step size for faster movement
//    public static int minStepSize = 10; // Minimum step size (slows down near target)
//
//    private int currentTarget = 0;
//
//    boolean wasAPressed = false;
//    boolean wasBPressed = false;
//
//    @Override
//    public void init() {
//        extender = new Extender(hardwareMap, "extendB");
//        dashboard = FtcDashboard.getInstance();
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void loop() {
//        // Update PID values from dashboard
//        extender.setPID(kP, kI, kD);
//
//        int currentPosition = extender.getCurrentPosition();
//
//        // Move towards target in smooth steps
//        if (gamepad1.a && !wasAPressed) {
//            currentTarget = targetPos; // Set new target position
//        } else if (gamepad1.b && !wasBPressed) {
//            currentTarget = 0; // Reset position
//        }
//
//        // **Gradual movement logic**
//        if (currentPosition < currentTarget) {
//            int distance = currentTarget - currentPosition;
//            int step = Math.max(minStepSize, Math.min(stepSize, distance / 2)); // Adjust step size
//            extender.setPosition(currentPosition + step);
//        } else if (currentPosition > currentTarget) {
//            int distance = currentPosition - currentTarget;
//            int step = Math.max(minStepSize, Math.min(stepSize, distance / 2)); // Adjust step size
//            extender.setPosition(currentPosition - step);
//        }
//
//        extender.update();
//
//        wasAPressed = gamepad1.a;
//        wasBPressed = gamepad1.b;
//
//        // Send telemetry data to FTC Dashboard
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Target Position", currentTarget);
//        packet.put("Current Position", currentPosition);
//        packet.put("At Target?", extender.isAtTarget());
//        packet.put("kP", kP);
//        packet.put("kI", kI);
//        packet.put("kD", kD);
//        packet.put("Step Size", stepSize);
//        dashboard.sendTelemetryPacket(packet);
//
//        // Also send to driver station telemetry
//        telemetry.addData("Target Position", currentTarget);
//        telemetry.addData("Current Position", currentPosition);
//        telemetry.addData("At Target?", extender.isAtTarget());
//        telemetry.addData("kP", kP);
//        telemetry.addData("kI", kI);
//        telemetry.addData("kD", kD);
//        telemetry.addData("Step Size", stepSize);
//        telemetry.update();
//    }
//}
