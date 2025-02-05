//package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;
//
//import com.qualcomm.robotcore.eventloop.opmode.*;
//import org.firstinspires.ftc.teamcode.utils.ArmTtest;
//
//@TeleOp(name = "armtest")
//public class armtest extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        ArmTtest arm;
//        waitForStart();
//        if (opModeIsActive()) {
//            arm = new ArmTtest(hardwareMap);
//            telemetry.addData("Status", "Initialized");
//            // Pre-run
//            while (opModeIsActive()) {
//                if (gamepad1.a) {
//                    arm.setPosition(1000);  // Move to outPose
//                }
//                else if (gamepad1.b) {
//                    arm.setPosition(0);  // Move back to zeroPose
//                }
//
//                arm.update(); // Call update to actually move the motor
//
//                telemetry.addData("Arm Position", arm.getCurrentPosition());
//                telemetry.addData("At Target?", arm.isAtTarget());
//                telemetry.update();
//            }
//        }
//    }
//}
