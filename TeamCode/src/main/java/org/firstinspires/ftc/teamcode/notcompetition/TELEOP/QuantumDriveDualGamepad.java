package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

@TeleOp(name = "QuantumDriveDualGamepad")
public class QuantumDriveDualGamepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor armLift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor armLift2 = hardwareMap.dcMotor.get("lift2");

        TouchSensor touch = hardwareMap.touchSensor.get("touch");

        ServoImplEx clawServo = (ServoImplEx)hardwareMap.servo.get("claw");
        ServoImplEx wristServo = (ServoImplEx)hardwareMap.servo.get("wrist");
        ServoImplEx arm1Servo = (ServoImplEx)hardwareMap.servo.get("arm1");
        ServoImplEx arm0Servo = (ServoImplEx)hardwareMap.servo.get("arm0");
        ServoImplEx liftUpServo = (ServoImplEx) hardwareMap.servo.get("liftUp");
        ServoImplEx liftUpLServo = (ServoImplEx) hardwareMap.servo.get("liftUpL");
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        liftUpServo.setPwmRange(pwmRange);
        liftUpLServo.setPwmRange(pwmRange);
        clawServo.setPwmRange(pwmRange);
        wristServo.setPwmRange(pwmRange);
        arm1Servo.setPwmRange(pwmRange);
        arm0Servo.setPwmRange(pwmRange);
        liftUpServo.setDirection(ServoImplEx.Direction.REVERSE);
        int lastPos1 = 0;
        int lastPos2 = 0;

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

        double deadzone = 0.1; // Adjust this value for the deadzone of the joysticks

        // armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Constants
        int liftOutTarget = -2200; // Maximum extension position
        int liftInTarget = 0;      // Fully retracted position
        double liftHoldPower = 0.3; // Power to hold the current position

        // Inverse kinematics constants
        final double MAX_LIFT_HEIGHT = 1.0;  // Max lift servo position
        final double MIN_LIFT_HEIGHT = 0.0;  // Min lift servo position
        final double MAX_ARM_ANGLE = 1.0;    // Max arm servo position
        final double MIN_ARM_ANGLE = 0.0;    // Min arm servo position
        final double DEADZONE = 0.1;         // Threshold for joystick input

        double liftSpeedMultiplier = 0.01; // Speed of lift adjustments
        double armSpeedMultiplier = 0.01;  // Speed of arm adjustments

        // Default positions and constants
        double liftPosition = 0.5; // Default lift position
        double arm0Position = 0.5; // Default arm0 position
        double arm1Position = 0.5; // Default arm1 position

        // Servo poses for clip on the wall
        double[] clipWall = {0.5105, 0, 0.228, 0.679, 0.65};
        double[] clipBar = {0.692, 0.789, 0.4995, 0.666, 0.35};
        double[] piecePick1 = {0.692,0.4, 0.4995, 0.666, 0.35};
        double[] piecePick2 = {0.7, 0, 0.228, 0.679, 0.35};
        int currentLiftPosition1, currentLiftPosition2;

        boolean pickingMode = false;  // Flag for gamepiece picking mode

        // Main loop: run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            // dn
            boolean liftOut = gamepad2.left_bumper;
            boolean liftIn = gamepad2.left_trigger > 0.1;
            boolean liftUp = gamepad2.right_bumper;
            boolean liftDown = gamepad2.right_trigger > 0.1;

            double liftJoystick = -gamepad2.left_stick_y; // Y-axis for lift (inverted)
            double armJoystick = gamepad2.left_stick_x;   // X-axis for arm
            double Sy = gamepad2.right_stick_y;
            double Sx = -gamepad2.right_stick_x * 1.1;

            if (gamepad2.start) {
                pickingMode = !pickingMode;
                sleep(300); // Debounce delay
            }

            if (pickingMode) {
                // Adjust lift position based on joystick input
                if (Math.abs(liftJoystick) > 0.1) { // Deadzone
                    liftPosition += liftJoystick * liftSpeedMultiplier;
                }

                // Adjust arm positions to keep claw perpendicular to ground
                if (Math.abs(armJoystick) > 0.1) { // Deadzone
                    arm0Position += armJoystick * armSpeedMultiplier;
                    arm1Position -= armJoystick * armSpeedMultiplier; // Opposite direction for perpendicularity
                }

                // Constrain positions to valid ranges
                liftPosition = Math.max(0.0, Math.min(1.0, liftPosition));
                arm0Position = Math.max(0.0, Math.min(1.0, arm0Position));
                arm1Position = Math.max(0.0, Math.min(1.0, arm1Position));

                // Set servo positions
                liftUpServo.setPosition(liftPosition);
                liftUpLServo.setPosition(liftPosition);
                arm0Servo.setPosition(arm0Position);
                arm1Servo.setPosition(arm1Position);

            }

            if (gamepad2.dpad_up) {
                double currentArm0Pos = arm0Servo.getPosition();
                arm0Servo.setPosition(currentArm0Pos+0.01);
            } else if (gamepad2.dpad_down) {
                double currentArm0Pos = arm0Servo.getPosition();
                arm0Servo.setPosition(currentArm0Pos-0.01);
            }
            if (gamepad2.dpad_left) {
                double currentArm1Pos = arm1Servo.getPosition();
                arm1Servo.setPosition(currentArm1Pos+0.01);
            } else if (gamepad2.dpad_right) {
                double currentArm1Pos = arm1Servo.getPosition();
                arm1Servo.setPosition(currentArm1Pos-0.01);
            }
            if (gamepad2.a) {
                // Grab the piece
                clawServo.setPosition(0.65);
            } else if (gamepad2.b) {
                // Extract the piece
                clawServo.setPosition(0.35);
            }

            if (gamepad2.x) {
                // Rotate CCW
                double currentWristPos = wristServo.getPosition();
                wristServo.setPosition(currentWristPos-0.01);
            }
            else if (gamepad2.y) {
                // Rotate CW
                double currentWristPos = wristServo.getPosition();
                wristServo.setPosition(currentWristPos+0.01);
            }
            if (liftUp){
                double currentLiftPos = liftUpServo.getPosition();
                liftUpServo.setPosition(currentLiftPos+0.05);
                double currentLiftPosL = liftUpLServo.getPosition();
                liftUpLServo.setPosition(currentLiftPosL+0.05);
            }
            if (liftDown){
                double currentLiftPos = liftUpServo.getPosition();
                liftUpServo.setPosition(currentLiftPos-0.05);
                double currentLiftPosL = liftUpLServo.getPosition();
                liftUpLServo.setPosition(currentLiftPosL-0.05);
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

            if (Math.abs(y) < deadzone && Math.abs(x) < deadzone && Math.abs(rx) < deadzone) {
                // Set target powers to 0 to stop motors immediately
                frontLeftTargetPower = 0;
                backLeftTargetPower = 0;
                frontRightTargetPower = 0;
                backRightTargetPower = 0;
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }else if (Math.abs(Sy) > deadzone && Math.abs(Sx) > deadzone) {
                // Calculate target powers based on gamepad input
                double denominator = max(Math.abs(Sy) + Math.abs(Sx), 1);
                frontLeftTargetPower = 1.0*(Sy + Sx) / denominator;
                backLeftTargetPower = (Sy - Sx) / denominator;
                frontRightTargetPower = 1.0*(Sy - Sx) / denominator;
                backRightTargetPower = (Sy + Sx) / denominator;
            } else {
                // Calculate target powers based on gamepad input
                double denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftTargetPower = 1.0*(y + x + rx) / denominator;
                backLeftTargetPower = (y - x + rx) / denominator;
                frontRightTargetPower = 1.0*(y - x - rx) / denominator;
                backRightTargetPower = (y + x - rx) / denominator;
            }

            if (!pickingMode) {
                if (gamepad2.right_stick_y > 0.5) {
                    armLift1.setTargetPosition(0); // Initial position is 0
                    armLift1.setPower(1);
                    armLift2.setTargetPosition(0); // Initial position is 0
                    armLift2.setPower(1);
                    armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftUpServo.setPosition(clipWall[0]);
                    liftUpLServo.setPosition(clipWall[0]);
                    arm0Servo.setPosition(clipWall[1]);
                    arm1Servo.setPosition(clipWall[2]);
                    wristServo.setPosition(clipWall[3]);
                    clawServo.setPosition(clipWall[4]);

                } else if (gamepad2.right_stick_y < -0.5) {
                    liftUpServo.setPosition(clipBar[0]);
                    liftUpLServo.setPosition(clipBar[0]);
                    arm0Servo.setPosition(clipBar[1]);
                    arm1Servo.setPosition(clipBar[2]);
                    wristServo.setPosition(clipBar[3]);
                    clawServo.setPosition(clipBar[4]);

                } else if (gamepad2.right_stick_x > 0.5) {
                    liftUpServo.setPosition(piecePick1[0]);
                    liftUpLServo.setPosition(piecePick1[0]);
                    arm0Servo.setPosition(piecePick1[1]);
                    arm1Servo.setPosition(piecePick1[2]);
                    wristServo.setPosition(piecePick1[3]);
                    clawServo.setPosition(piecePick1[4]);

                } else if (gamepad2.right_stick_x < -0.5) {
                    liftUpServo.setPosition(piecePick2[0]);
                    liftUpLServo.setPosition(piecePick2[0]);
                }
            }

            telemetry.addData("Servo Positions", "----");
            telemetry.addData("Lift1 Position", currentLiftPosition1);
            telemetry.addData("Lift2 Position", currentLiftPosition2);
            telemetry.addData("LiftUp Servo", liftUpServo.getPosition());
            telemetry.addData("LiftUpL Servo", liftUpLServo.getPosition());
            telemetry.addData("Arm0 Servo", arm0Servo.getPosition());
            telemetry.addData("Arm1 Servo", arm1Servo.getPosition());
            telemetry.addData("Wrist Servo", wristServo.getPosition());
            telemetry.addData("Claw Servo", clawServo.getPosition());

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
