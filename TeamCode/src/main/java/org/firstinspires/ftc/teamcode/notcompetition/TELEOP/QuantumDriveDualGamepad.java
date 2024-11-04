package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
                clawServo.setPosition(0.45);
            } else if (gamepad2.b) {
                // Extract the piece
                clawServo.setPosition(0);
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
                liftUpServo.setPosition(currentLiftPos-0.01);
                double currentLiftPosL = liftUpLServo.getPosition();
                liftUpLServo.setPosition(currentLiftPosL+0.01);
            }
            if (liftDown){
                double currentLiftPos = liftUpServo.getPosition();
                liftUpServo.setPosition(currentLiftPos+0.01);
                double currentLiftPosL = liftUpLServo.getPosition();
                liftUpLServo.setPosition(currentLiftPosL-0.01);
            }

            int armLift1CurrentPosition = armLift1.getCurrentPosition();
            int armLift2CurrentPosition = armLift2.getCurrentPosition();
            if (liftOut) {
                armLift1.setTargetPosition(min(armLift1CurrentPosition - 50,-1600));
                armLift1.setPower(1);
                armLift2.setTargetPosition(min(armLift2CurrentPosition - 50,-1600));
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lastPos1 = armLift1CurrentPosition;
                lastPos2 = armLift2CurrentPosition;
            } else if (liftIn) {
                armLift1.setTargetPosition(max(armLift1CurrentPosition + 50, 0)); // Initial position is 0
                armLift1.setPower(1);
                armLift2.setTargetPosition(max(armLift2CurrentPosition + 50, 0)); // Initial position is 0
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lastPos1 = armLift1CurrentPosition;
                lastPos2 = armLift2CurrentPosition;
            } else {
                // Hold position when button is not pressed
                armLift1.setTargetPosition(lastPos1);
                armLift1.setPower(0.3); // Reduced power for holding
                armLift2.setTargetPosition(lastPos2);
                armLift2.setPower(0.3); // Reduced power for holding
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            } else {
                // Calculate target powers based on gamepad input
                double denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftTargetPower = 1.1*(y + x + rx) / denominator;
                backLeftTargetPower = (y - x + rx) / denominator;
                frontRightTargetPower = 1.1*(y - x - rx) / denominator;
                backRightTargetPower = (y + x - rx) / denominator;
            }

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
