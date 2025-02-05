package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.ArmTtest;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "ArmT PID Dashboard", group = "Test")
@Config
public class ArmTtestDashboard extends OpMode {
    private ArmTtest armT;
    private FtcDashboard dashboard;

    // Tunable PID values in FTC Dashboard
    public static double kP = 0.001;
    public static double kI = 0.001;
    public static double kD = 0.00001;

    boolean wasAPressed = false;
    boolean wasBPressed = false;

    @Override
    public void init() {
        armT = new ArmTtest(hardwareMap, "armT");
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Update PID values from dashboard
        armT.setPID(kP, kI, kD);

        // Control arm position with gamepad
        if (gamepad1.a && !wasAPressed) {
            armT.setPosition(2000);// Move to target position
        }
        else if (gamepad1.b && !wasBPressed) {
            armT.setPosition(0); // Reset position
        }

        armT.update();

        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;

        // Send telemetry data to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Position", 2000);
        packet.put("Current Position", armT.getCurrentPosition());
        packet.put("At Target?", armT.isAtTarget());
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);
        dashboard.sendTelemetryPacket(packet);

        // Also send to driver station telemetry
        telemetry.addData("Target Position", 2000);
        telemetry.addData("Current Position", armT.getCurrentPosition());
        telemetry.addData("At Target?", armT.isAtTarget());
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}

