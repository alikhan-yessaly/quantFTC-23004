//package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.utils.ArmTtest;
//
//@TeleOp(name = "ArmT PID Test", group = "Test")
//public class ArmTtestTeleOp extends OpMode {
//    private ArmTtest armT;
//
//    @Override
//    public void init() {
//        armT = new ArmTtest(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void loop() {
//        // Move arm to preset positions using gamepad buttons
//        if (gamepad1.a) {
//            armT.setPosition(2000); // Move to a high position
//        } else if (gamepad1.b) {
//            armT.setPosition(0); // Move back to the starting position
//        }
//
//        armT.update(); // Continuously update PID control
//
//        // Display telemetry data
//        telemetry.addData("Target Position", 2000);
//        telemetry.addData("Current Position", armT.getCurrentPosition());
//        telemetry.addData("At Target?", armT.isAtTarget());
//        telemetry.update();
//    }
//}
