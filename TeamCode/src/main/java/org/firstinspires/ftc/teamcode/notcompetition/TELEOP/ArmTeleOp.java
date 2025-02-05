package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.ArmTAdvanced;

@TeleOp(name = "ArmTAdvancedTest", group = "Test")
public class ArmTeleOp extends OpMode {
    private ArmTAdvanced armController;

    @Override
    public void init() {
        // Initialize arm controller with hardware map
        armController = new ArmTAdvanced(hardwareMap);  // Use actual motor name in configuration
        armController.autoTune();
    }

    @Override
    public void loop() {
        // Increase target angle with Gamepad buttons
        if (gamepad1.dpad_up) {
            armController.setTargetAngle(armController.getTargetAngle() + 5);  // Increase angle by 5 degrees
        }
        if (gamepad1.dpad_down) {
            armController.setTargetAngle(armController.getTargetAngle() - 5);  // Decrease angle by 5 degrees
        }

        // Set predefined positions with buttons
        if (gamepad1.a) armController.setTargetAngle(0);    // Arm to 0 degrees
        if (gamepad1.b) armController.setTargetAngle(45);   // Arm to 45 degrees
        if (gamepad1.y) armController.setTargetAngle(90);   // Arm to 90 degrees

        // Call update to continuously adjust motor power
        armController.update();

        // Send telemetry data to the Driver Station
        telemetry.addData("Target Angle (deg)", armController.getTargetAngle());
        telemetry.update();
    }
}