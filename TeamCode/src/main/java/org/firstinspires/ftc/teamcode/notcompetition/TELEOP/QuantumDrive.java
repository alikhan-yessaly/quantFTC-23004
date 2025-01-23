package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "QuantumDrive")
public class QuantumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor armLift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor armLift2 = hardwareMap.dcMotor.get("lift2");

        TouchSensor touch = hardwareMap.touchSensor.get("touch");

        ServoImplEx clawTServo = (ServoImplEx)hardwareMap.servo.get("clawT");
        ServoImplEx clawBServo = (ServoImplEx)hardwareMap.servo.get("clawB");
        ServoImplEx wristBServo = (ServoImplEx)hardwareMap.servo.get("wristB");
        ServoImplEx wristTServo = (ServoImplEx)hardwareMap.servo.get("wristT");
        ServoImplEx armBServo = (ServoImplEx) hardwareMap.servo.get("armB");
        ServoImplEx armTServo = (ServoImplEx) hardwareMap.servo.get("armT");
        ServoImplEx extendRServo = (ServoImplEx) hardwareMap.servo.get("extendR");
        ServoImplEx extendLServo = (ServoImplEx) hardwareMap.servo.get("extendL");

        extendLServo.setDirection(ServoImplEx.Direction.REVERSE);
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        extendLServo.setPwmRange(pwmRange);
        extendRServo.setPwmRange(pwmRange);
        armBServo.setPwmRange(pwmRange);
        armTServo.setPwmRange(pwmRange);
        clawTServo.setPwmRange(pwmRange);
        clawBServo.setPwmRange(pwmRange);
        wristBServo.setPwmRange(pwmRange);
        wristTServo.setPwmRange(pwmRange);

        boolean clawTclosed = false;
        boolean clawBclosed = false;
        boolean bWasPressed = false;
        boolean aWasPressed = false;
        boolean armBButtonPressed = false;
        boolean modeEnabled = false;
        boolean modeButtonPressed = false;
        boolean dpadUpIsPressed = false;


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        if (isStopRequested()) return;

        double rampUpRate = 0.1; // Adjust this value for faster/slower ramp-up
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        double frontLeftTargetPower;
        double backLeftTargetPower;
        double frontRightTargetPower;
        double backRightTargetPower;
        double armBPosition = 0.2;;
        int wristBPositionState = 0;


        double deadzone = 0.1; // Adjust this value for the deadzone of the joysticks

        // armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Constants
        int liftOutTarget = -1900; // Maximum extension position
        int liftInTarget = 0;      // Fully retracted position
        double liftHoldPower = 0.3; // Power to hold the current position

        int currentLiftPosition1, currentLiftPosition2;

        clawTServo.setPosition(0.35);
        clawBServo.setPosition(0.35);
        armTServo.setPosition(0.25);
        extendLServo.setPosition(0.25);
        extendRServo.setPosition(0.25);
        wristBServo.setPosition(0.55);
        wristTServo.setPosition(0.5);


        // Main loop: run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y * 0.85;
            double x = (gamepad1.left_stick_x * 1.1) * 0.85;
            double rx = (-gamepad1.right_stick_x) * 0.85;

            double y2 = gamepad2.left_stick_y;
            double x2 = gamepad2.left_stick_x * 0.5;

            // dn
            boolean liftOut = gamepad2.left_bumper;
            boolean liftIn = gamepad2.left_trigger > 0.1;
            boolean extendOut = gamepad2.right_bumper;
            boolean extendIn = gamepad2.right_trigger > 0.2;

            if(Math.abs(y2) > 0.1) {
                double currentExtendPosR = extendRServo.getPosition();
                extendRServo.setPosition(currentExtendPosR-0.01*y2);
                double currentExtendPosL = extendLServo.getPosition();
                extendLServo.setPosition(currentExtendPosL-0.01*y2);
            }



            if(gamepad2.guide && !modeButtonPressed){
                modeEnabled = !modeEnabled;

                modeButtonPressed = true;
            } else if(!gamepad2.guide){
                modeButtonPressed = false;
            }

            if(modeEnabled){
                if(gamepad2.dpad_up && !dpadUpIsPressed){


                }
                else if(gamepad2.dpad_down){
                    armTServo.setPosition(0.88);
                    wristTServo.setPosition(0.5);
                }
                dpadUpIsPressed = gamepad2.dpad_up;
            }
            else {
                if (gamepad2.dpad_up) {
                    double currentArmTPos = armTServo.getPosition();
                    armTServo.setPosition(currentArmTPos+0.01);
                } else if (gamepad2.dpad_down) {
                    double currentArmTPos = armTServo.getPosition();
                    armTServo.setPosition(currentArmTPos-0.01);
                }
            }

            if (gamepad2.dpad_left) {
                wristTServo.setPosition(0.5);
            } else if (gamepad2.dpad_right) {
                wristTServo.setPosition(0.0);
            }

            if (gamepad2.a && !aWasPressed) {
                clawTclosed = !clawTclosed;
                if (clawTclosed) {
                    clawTServo.setPosition(0.65);
//                    clawBServo.setPosition(0.35);
                } else {
                    clawTServo.setPosition(0.35);
                }
            }
            aWasPressed = gamepad2.a;


            // Claw control logic with arm position adjustments
            if (gamepad2.b && !bWasPressed) {
                clawBclosed = !clawBclosed; // Toggle claw state

                if (clawBclosed) {
                    // If the claw is closing and armBPosition is 0.15, lower the arm
                    if (armBPosition == 0.15) {
                        armBPosition = 0; // Lower arm
                        armBServo.setPosition(armBPosition); // Apply new arm position
                        sleep(200); // Wait for the arm to lower
                    }

                    
                    clawBServo.setPosition(0.65); // Close claw
                    sleep(200); // Wait for claw to fully close

                    if (armBPosition==0) {
                        armBPosition = 0.15;
                        armBServo.setPosition(armBPosition);
                    }
                } else {
                    // Open the claw
                    clawBServo.setPosition(0.35);
                }
            }

            // Update the button press state to avoid repeated toggling
            bWasPressed = gamepad2.b;


            if (gamepad2.y && !armBButtonPressed) {
                // Cycle through positions when button is pressed
                if (armBPosition == 0.15) {
                    armBPosition = 1; // Move to transfer position++
                    wristBServo.setPosition(0.5);
                    wristTServo.setPosition(0.5);
                    armTServo.setPosition(0.1495);
                    clawBServo.setPosition(0.65);
                    clawTServo.setPosition(0.65);
                    extendLServo.setPosition(0.25);
                    extendRServo.setPosition(0.25);
                } else {
                    armBPosition = 0.15; // Move back to starting position
                }

                // Update the button press state to avoid multiple triggers
                armBButtonPressed = true;
            } else if (!gamepad2.y) {
                // Reset button press state when button is released
                armBButtonPressed = false;
            }

           // Set the position of armB servo
            armBServo.setPosition(armBPosition);


            if (gamepad2.x) {
                // Wait a small amount of time to avoid button spamming
                sleep(200);  // Adjust the sleep time if needed

                // Increment position state, looping back to 0 when reaching 3
                wristBPositionState = (wristBPositionState + 1) % 4;

                // Set wristB position based on current state
                switch (wristBPositionState) {
                    case 0: // Starting position (0 degrees)
                        wristBServo.setPosition(0.5);  // Adjust this value for the starting position
                        break;
                    case 1: // 45 degrees left
                        wristBServo.setPosition(0.3);  // Adjust this value for 45 degrees to the left
                        break;
                    case 2: // 45 degrees right
                        wristBServo.setPosition(0.7);  // Adjust this value for 45 degrees to the right
                        break;
                    case 3: // 45 degrees right
                        wristBServo.setPosition(0.85);  // Adjust this value for 45 degrees to the right
                        break;
                }
            }


//            if (gamepad2.dpad_left) {
//                // Wait a small amount of time to avoid button spamming
//                sleep(200);  // Adjust the sleep time if needed
//
//                // Increment position state, looping back to 0 when reaching 3
//                wristTPositionState = (wristTPositionState + 1) % 3;
//
//                // Set wristB position based on current state
//                switch (wristTPositionState) {
//                    case 0: // Starting position (0 degrees)
//                        wristTServo.setPosition(0.5);  // Adjust this value for the starting position
//                        break;
//                    case 1: // 45 degrees left
//                        wristTServo.setPosition(0.35);  // Adjust this value for 45 degrees to the left
//                        break;
//                    case 2: // 45 degrees right
//                        wristTServo.setPosition(0.65);  // Adjust this value for 45 degrees to the right
//                        break;
//                }
//            }


            if (extendOut){
                double currentLiftPos = extendRServo.getPosition();
                extendRServo.setPosition(currentLiftPos+0.05);
                double currentLiftPosL = extendLServo.getPosition();
                extendLServo.setPosition(currentLiftPosL+0.05);
            }
            if (extendIn){
                double currentLiftPos = extendRServo.getPosition();
                extendRServo.setPosition(currentLiftPos-0.05);
                double currentLiftPosL = extendLServo.getPosition();
                extendLServo.setPosition(currentLiftPosL-0.05);
            }

            currentLiftPosition1 = armLift1.getCurrentPosition();
            currentLiftPosition2 = armLift2.getCurrentPosition();

            if (liftOut) {
                // Run motors to top position
                armLift1.setTargetPosition(liftOutTarget);
                armLift2.setTargetPosition(liftOutTarget);
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (liftIn && !touch.isPressed()) {
                // Run motors to bottom position unless touch sensor is pressed
                armLift1.setTargetPosition(liftInTarget);
                armLift2.setTargetPosition(liftInTarget);
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                // Hold current position
                armLift1.setTargetPosition(currentLiftPosition1);
                armLift2.setTargetPosition(currentLiftPosition2);
                armLift1.setPower(liftHoldPower);
                armLift2.setPower(liftHoldPower);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (touch.isPressed()) {
                // Reset encoders when the lift is fully retracted
                armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift1.setTargetPosition(liftInTarget);
                armLift2.setTargetPosition(liftInTarget);
            }

            if (Math.abs(y) < deadzone && Math.abs(x) < deadzone && Math.abs(rx) < deadzone && Math.abs(x2)<deadzone) {
                // Stop motors immediately if within the deadzone
                frontLeftTargetPower = 0;
                backLeftTargetPower = 0;
                frontRightTargetPower = 0;
                backRightTargetPower = 0;
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
            else {
                // Separate the logic for strafing (x) and turning (rx)
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) + Math.abs(x2), 1);

                // Corrected motor power calculations
                frontLeftTargetPower = (y + x + rx + x2) / denominator;
                backLeftTargetPower = (y - x + rx - x2) / denominator;
                frontRightTargetPower = (y - x - rx - x2) / denominator;
                backRightTargetPower = (y + x - rx + x2) / denominator;
            }



            telemetry.addData("Servo Positions", "----");
            telemetry.addData("Lift1 Position", currentLiftPosition1);
            telemetry.addData("Lift2 Position", currentLiftPosition2);
            telemetry.addData("ClawT Position", clawTServo.getPosition());
            telemetry.addData("ClawB Position", clawBServo.getPosition());
            telemetry.addData("WristB Position", wristBServo.getPosition());
            telemetry.addData("WristT Position", wristTServo.getPosition());
            telemetry.addData("ArmB Position", armBServo.getPosition());
            telemetry.addData("ArmT Position", armTServo.getPosition());
            telemetry.addData("Front Left Power", frontLeftTargetPower);
            telemetry.addData("Back Left Power", backLeftTargetPower);
            telemetry.addData("Front Right Power", frontRightTargetPower);
            telemetry.addData("Back Right Power", backRightTargetPower);
            telemetry.addData("extendL", extendLServo.getPosition());
            telemetry.addData("extendR", extendRServo.getPosition());
            telemetry.update();

            telemetry.update();

            // Ramp up motor powers towards target powers
            frontLeftPower += rampUpRate * (frontLeftTargetPower - frontLeftPower);
            backLeftPower += rampUpRate * (backLeftTargetPower - backLeftPower);
            frontRightPower += rampUpRate * (frontRightTargetPower - frontRightPower);
            backRightPower += rampUpRate * (backRightTargetPower - backRightPower);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        }
    }
}
