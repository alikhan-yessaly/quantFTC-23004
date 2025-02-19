//package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//
//
//import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
////import org.firstinspires.ftc.teamcode.utils.ArmT;
//import org.firstinspires.ftc.teamcode.utils.Extender;
//
//
//@TeleOp(name = "QuantumDrive")
//public class QuantumDrive extends LinearOpMode {
//    /*    private Follower follower;*/
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        DcMotor armLift1 = hardwareMap.dcMotor.get("lift1");
//        DcMotor armLift2 = hardwareMap.dcMotor.get("lift2");
//        DcMotor extendB = hardwareMap.dcMotor.get("extendB");
////        DcMotor armT = hardwareMap.dcMotor.get("armT");
////        ArmT armT1 = new ArmT(hardwareMap, "armT");
////        Extender extendB1 = new Extender(hardwareMap, "extendB");
//
//        TouchSensor touch = hardwareMap.touchSensor.get("touch");
//
////        follower = new Follower(hardwareMap);
//
//        ServoImplEx clawTServo = (ServoImplEx) hardwareMap.servo.get("clawT");
//        ServoImplEx clawBServo = (ServoImplEx) hardwareMap.servo.get("clawB");
//        ServoImplEx wristBServo = (ServoImplEx) hardwareMap.servo.get("wristB");
//        ServoImplEx wristTServo = (ServoImplEx) hardwareMap.servo.get("wristT");
//        ServoImplEx armBServo = (ServoImplEx) hardwareMap.servo.get("armB");
//
//        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
//
//        armBServo.setPwmRange(pwmRange);
//        clawTServo.setPwmRange(pwmRange);
//        clawBServo.setPwmRange(pwmRange);
//        wristBServo.setPwmRange(pwmRange);
//        wristTServo.setPwmRange(pwmRange);
//
//        boolean clawTclosed = false;
//        boolean clawBclosed = false;
//        boolean bWasPressed = false;
//        boolean aWasPressed = false;
//        boolean armBButtonPressed = false;
//        boolean modeEnabled = false;
//        boolean modeButtonPressed = false;
//        boolean dpadUpIsPressed = false;
//        boolean dpadLeftIsPressed = false;
//        boolean transferPiece = false;
//        boolean transferStarted = false;
//        boolean liftFall = false;
//
//        int transferPos = -500;
//
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        extendB.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armLift2.setDirection(DcMotorSimple.Direction.REVERSE);
//        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////        armT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        armT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        extendB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        extendB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        double rampUpRate = 0.1; // Adjust this value for faster/slower ramp-up
//        double frontLeftPower = 0;
//        double backLeftPower = 0;
//        double frontRightPower = 0;
//        double backRightPower = 0;
//        double frontLeftTargetPower;
//        double backLeftTargetPower;
//        double frontRightTargetPower;
//        double backRightTargetPower;
//        double armBPosition = 0.2;
//        ;
//        int wristBPositionState = 0;
//        int targetPosition = 0;
//        int liftUpPosition = -1250;
//
//
//        double deadzone = 0.1; // Adjust this value for the deadzone of the joysticks
//
//        // armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        armT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        extendB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        armT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        extendB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //armBServo.setDirection(Servo.Direction.REVERSE);
//
//
//        // Constants
//        int liftOutTarget = -2500; // Maximum extension position
//        int liftInTarget = 0;      // Fully retracted position
//        double liftHoldPower = 0.5; // Power to hold the current position
//
//        int currentLiftPosition1, currentLiftPosition2;
//
//        clawTServo.setPosition(0.35);
//        clawBServo.setPosition(0.35);
//        wristBServo.setPosition(0.5);
//        wristTServo.setPosition(0.3);
//
//
//        // Main loop: run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            double y = gamepad2.left_stick_y * 0.85;
//            double x = (gamepad2.left_stick_x * 1.1) * 0.85;
//            double rx = (-gamepad2.right_stick_x) * 0.85;
//
//            double y2 = gamepad2.right_stick_y;
//            double x2 = 0;
//
//            /* follower.update();*/
//
//
//            // dn
//            boolean liftOut = gamepad2.left_bumper;
//            boolean liftIn = gamepad2.left_trigger > 0.1;
//            boolean extendOut = gamepad2.right_bumper;
//            boolean extendIn = gamepad2.right_trigger > 0.2;
//
//
//            if (gamepad2.guide && !modeButtonPressed) {
//                modeEnabled = !modeEnabled;
//                modeButtonPressed = true;
//            } else if (!gamepad2.guide) {
//                modeButtonPressed = false;
//            }
//
//            if (modeEnabled) {
//                if (gamepad2.dpad_up && !dpadUpIsPressed) {
//                    if ((armLift1.getCurrentPosition() == liftUpPosition) && (armLift2.getCurrentPosition() == liftUpPosition)) {
//                        armLift1.setTargetPosition(liftUpPosition);
//                        armLift2.setTargetPosition(liftUpPosition);
//                        armLift1.setPower(0.5);
//                        armLift2.setPower(0.5);
//                        armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    } else {
//                        while (armLift1.getCurrentPosition() > liftUpPosition && armLift2.getCurrentPosition() > liftUpPosition) {
//                            armLift1.setTargetPosition(targetPosition);
//                            armLift2.setTargetPosition(targetPosition);
//                            armLift1.setPower(1);
//                            armLift2.setPower(1);
//                            armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            targetPosition -= 50;
//                        }
//                    }
//                } else if (gamepad2.dpad_down) {
////                    armT.setTargetPosition(200);
////                    armT.setPower(1);
////                    armT1.setPosition(200);
//                    wristTServo.setPosition(0.3);
//                }
//                dpadUpIsPressed = gamepad2.dpad_up;
//
//                if ((gamepad2.dpad_left && !dpadLeftIsPressed)) {
////                    armT.setTargetPosition(200);
////                    armT.setPower(1);
////                    armT1.setPosition(200);
//                }
//                dpadLeftIsPressed = gamepad2.dpad_left;
//            } else {
//                if (gamepad2.dpad_up) {
////                    int currentArmTPos = armT.getCurrentPosition();
////                    armT.setTargetPosition(5000);
////                    armT.setPower(0.2);
////                    armT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    armT1.setPosition(5000);
//                } else if (gamepad2.dpad_down) {
////                    int currentArmTPos = armT.getCurrentPosition();
////                    armT.setTargetPosition(0);
////                    armT.setPower(0.2);
////                    armT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    armT1.setPosition(0);
//                } else {
////                    armT.setPower(0);
////                    armT1.stop();
//                }
//
//                if (gamepad2.dpad_left) {
//                    wristTServo.setPosition(0.5);
//                } else if (gamepad2.dpad_right) {
//                    wristTServo.setPosition(1);
//                }
//            }
//
//
//            if (gamepad2.a && !aWasPressed) {
//                clawTclosed = !clawTclosed;
//                if (clawTclosed) {
//                    clawTServo.setPosition(0.65);
////                    clawBServo.setPosition(0.35);
//                } else {
//                    clawTServo.setPosition(0.35);
//                }
//            }
//            aWasPressed = gamepad2.a;
//
//
//            // Claw control logic with arm position adjustments
//            if (gamepad2.b && !bWasPressed) {
//                clawBclosed = !clawBclosed; // Toggle claw state
//
//                if (clawBclosed) {
//                    // If the claw is closing and armBPosition is 0.15, lower the arm
//                    if (armBPosition == 0.22) {
//                        armBPosition = 0; // Lower arm
//                        armBServo.setPosition(armBPosition); // Apply new arm position
//                        sleep(200); // Wait for the arm to lower
//                    }
//
//
//                    clawBServo.setPosition(0.65); // Close claw
//                    sleep(200); // Wait for claw to fully close
//
//                    if (armBPosition == 0) {
//                        armBPosition = 0.22;
//                        armBServo.setPosition(armBPosition);
//                    }
//                } else {
//                    // Open the claw
//                    clawBServo.setPosition(0.35);
//                }
//            }
//
//            // Update the button press state to avoid repeated toggling
//            bWasPressed = gamepad2.b;
//
//
//            if (gamepad2.y && !armBButtonPressed) {
//                transferStarted = false;
//                // Cycle through positions when button is pressed
//                if (armBPosition == 0.22) {
//                    armBPosition = 1; // Move to transfer position+
//                    if (!transferStarted) {
//                        transferPiece = true;
//                    }
//                    wristBServo.setPosition(0.5);
//                    wristTServo.setPosition(0.3);
////                    armT.setTargetPosition(200);
////                    armT.setPower(1);
////                    armT1.setPosition(200);
//                    clawBServo.setPosition(0.65);
//
//                    clawTServo.setPosition(0.35);
//                    extendB.setTargetPosition(200);
//                    extendB.setPower(1);
////                    extendB1.setPosition(200);
//
//                } else {
//                    transferPiece = false;
//                    armBPosition = 0.22; // Move back to starting position
//                }
//
//                // Update the button press state to avoid multiple triggers
//                armBButtonPressed = true;
//            } else if (!gamepad2.y) {
//                // Reset button press state when button is released
//                armBButtonPressed = false;
//            }
//
//            // Set the position of armB servo
//            armBServo.setPosition(armBPosition);
//
//
//            if (gamepad2.x) {
//                // Wait a small amount of time to avoid button spamming
//                sleep(200);  // Adjust the sleep time if needed
//
//                // Increment position state, looping back to 0 when reaching 3
//                wristBPositionState = (wristBPositionState + 1) % 4;
//
//                // Set wristB position based on current state
//                switch (wristBPositionState) {
//                    case 0: // Starting position (0 degrees)
//                        wristBServo.setPosition(0.5);  // Adjust this value for the starting position
//                        break;
//                    case 1: // 45 degrees left
//                        wristBServo.setPosition(0.3);  // Adjust this value for 45 degrees to the left
//                        break;
//                    case 2: // 45 degrees right
//                        wristBServo.setPosition(0.7);  // Adjust this value for 45 degrees to the right
//                        break;
//                    case 3: // 45 degrees right
//                        wristBServo.setPosition(0.85);  // Adjust this value for 45 degrees to the right
//                        break;
//                }
//            }
//
//
//            if (extendOut) {
//                int currentExtendBPos = extendB.getCurrentPosition();
//                extendB.setTargetPosition(1500);
//                extendB.setPower(0.1);
//                extendB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                extendB1.setPosition(-1500);
//            } else {
//                extendB.setPower(0);
////                extendB1.stop();
//            }
//            if (extendIn) {
//                int currentExtendBPos = extendB.getCurrentPosition();
//                extendB.setTargetPosition(0);
//                extendB.setPower(0.1);
//                extendB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                extendB1.setPosition(0);
//            } else {
//                extendB.setPower(0);
////                extendB1.stop();
//
//                currentLiftPosition1 = armLift1.getCurrentPosition();
//                currentLiftPosition2 = armLift2.getCurrentPosition();
//
//                if (liftOut) {
//                    // Run motors to top position
//                    armLift1.setTargetPosition(liftOutTarget);
//                    armLift2.setTargetPosition(liftOutTarget);
//                    armLift1.setPower(1);
//                    armLift2.setPower(1);
//                    armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                } else if (liftIn && !touch.isPressed()) {
//                    transferStarted = true;
//                    // Run motors to bottom position unless touch sensor is pressed
//                    armLift1.setTargetPosition(liftInTarget);
//                    armLift2.setTargetPosition(liftInTarget);
//                    armLift1.setPower(1);
//                    armLift2.setPower(1);
//                    armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                } else if (transferPiece && !transferStarted) {
//                    // Hold current position
//                    if (currentLiftPosition1 > -170) {
//                        armLift1.setPower(0);
//                        armLift2.setPower(0);
//                    } else {
//                        armLift1.setPower(1);
//                        armLift2.setPower(1);
//                    }
//                    armLift1.setTargetPosition(transferPos);
//                    armLift2.setTargetPosition(transferPos);
//                    armLift1.setPower(1);
//                    armLift2.setPower(1);
//                    armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                } else {
//                    armLift1.setTargetPosition(currentLiftPosition1);
//                    armLift2.setTargetPosition(currentLiftPosition2);
//                    armLift1.setPower(liftHoldPower);
//                    armLift2.setPower(liftHoldPower);
//                    armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//
//                if (touch.isPressed()) {
//                    // Reset encoders when the lift is fully retracted
//                    armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    armLift1.setTargetPosition(liftInTarget);
//                    armLift2.setTargetPosition(liftInTarget);
//                }
//
//                if (Math.abs(y) < deadzone && Math.abs(x) < deadzone && Math.abs(rx) < deadzone && Math.abs(x2) < deadzone) {
//                    // Stop motors immediately if within the deadzone
//                    frontLeftTargetPower = 0;
//                    backLeftTargetPower = 0;
//                    frontRightTargetPower = 0;
//                    backRightTargetPower = 0;
//                    frontLeftMotor.setPower(0);
//                    backLeftMotor.setPower(0);
//                    frontRightMotor.setPower(0);
//                    backRightMotor.setPower(0);
//                } else {
//                    // Separate the logic for strafing (x) and turning (rx)
//                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) + Math.abs(x2), 1);
//
//                    // Corrected motor power calculations
//                    frontLeftTargetPower = (y + x + rx + x2) / denominator;
//                    backLeftTargetPower = (y - x + rx - x2) / denominator;
//                    frontRightTargetPower = (y - x - rx - x2) / denominator;
//                    backRightTargetPower = (y + x - rx + x2) / denominator;
//                }
//
//
//                telemetry.addData("Servo Positions", "----");
//                telemetry.addData("Lift1 Position", currentLiftPosition1);
//                telemetry.addData("Lift2 Position", currentLiftPosition2);
//                telemetry.addData("ClawT Position", clawTServo.getPosition());
//                telemetry.addData("ClawB Position", clawBServo.getPosition());
//                telemetry.addData("WristB Position", wristBServo.getPosition());
//                telemetry.addData("WristT Position", wristTServo.getPosition());
//                telemetry.addData("ArmB Position", armBServo.getPosition());
////                telemetry.addData("ArmT Position", armT1.getCurrentPosition());
//                telemetry.addData("Front Left Power", frontLeftTargetPower);
//                telemetry.addData("Back Left Power", backLeftTargetPower);
//                telemetry.addData("Front Right Power", frontRightTargetPower);
//                telemetry.addData("Back Right Power", backRightTargetPower);
//                telemetry.addData("extendB", extendB.getCurrentPosition());
////            telemetry.addData("X", follower.getPose().getX());
////            telemetry.addData("Y", follower.getPose().getY());
////            telemetry.addData(" Heading", follower.getPose().getHeading());
//                telemetry.update();
//
//
//                // Ramp up motor powers towards target powers
//                frontLeftPower += rampUpRate * (frontLeftTargetPower - frontLeftPower);
//                backLeftPower += rampUpRate * (backLeftTargetPower - backLeftPower);
//                frontRightPower += rampUpRate * (frontRightTargetPower - frontRightPower);
//                backRightPower += rampUpRate * (backRightTargetPower - backRightPower);
//
//                frontLeftMotor.setPower(frontLeftPower);
//                backLeftMotor.setPower(backLeftPower);
//                frontRightMotor.setPower(frontRightPower);
//                backRightMotor.setPower(backRightPower);
//
//
//            }
//        }
//    }
//}
