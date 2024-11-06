package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.LiftUp;
import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.Wrist;
import org.firstinspires.ftc.teamcode.utils.Claw;



@TeleOp(name = "QuantumDrive")
public class QuantumDrive extends LinearOpMode {
    boolean xPressedPreviously = false;
    @Override
    public void runOpMode(){
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor armLift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor armLift2 = hardwareMap.dcMotor.get("lift2");

        // Instantiate the custom servo classes
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        LiftUp liftUp = new LiftUp(hardwareMap);

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

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Main loop: run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            boolean liftOut = gamepad1.left_bumper;
            boolean liftIn = gamepad1.left_trigger > 0.1;
            boolean liftUpTrigger = gamepad1.right_bumper;
            boolean liftDownTrigger = gamepad1.right_trigger > 0.1;

            // Arm positioning using D-Pad
            if (gamepad1.dpad_up) {
                arm.increaseArm0Position(0.01);
            } else if (gamepad1.dpad_down) {
                arm.decreaseArm0Position(0.01);
            }
            if (gamepad1.dpad_left) {
                arm.increaseArm1Position(0.01);
            } else if (gamepad1.dpad_right) {
                arm.decreaseArm1Position(0.01);
            }

            // Claw control
            if (gamepad1.a) {
                claw.grab();
            } else if (gamepad1.b) {
                claw.release();
            }

            if (gamepad1.x && !xPressedPreviously) {
                wrist.cyclePosition();
                xPressedPreviously = true;
            } else if (!gamepad1.x) {
                xPressedPreviously = false;
            }

            // Lift positioning
            if (liftUpTrigger) {
                liftUp.raise(0.01);
            } else if (liftDownTrigger) {
                liftUp.lower(0.01);
            }

            // Display servo positions for debugging
            telemetry.addData("Arm0 Position", arm.getArm0Position());
            telemetry.addData("Arm1 Position", arm.getArm1Position());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("LiftUp Position", liftUp.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());

            // Arm lift control
            int armLift1CurrentPosition = armLift1.getCurrentPosition();
            int armLift2CurrentPosition = armLift2.getCurrentPosition();
            telemetry.addData("lift1:", armLift1CurrentPosition);
            telemetry.addData("lift2:", armLift2CurrentPosition);
            telemetry.update();

            if (liftOut) {
                armLift1.setTargetPosition(min(armLift1CurrentPosition - 50, -1600));
                armLift2.setTargetPosition(min(armLift2CurrentPosition - 50, -1600));
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lastPos1 = armLift1CurrentPosition;
                lastPos2 = armLift2CurrentPosition;
            } else if (liftIn) {
                armLift1.setTargetPosition(max(armLift1CurrentPosition + 50, 0));
                armLift2.setTargetPosition(max(armLift2CurrentPosition + 50, 0));
                armLift1.setPower(1);
                armLift2.setPower(1);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lastPos1 = armLift1CurrentPosition;
                lastPos2 = armLift2CurrentPosition;
            } else {
                armLift1.setTargetPosition(lastPos1);
                armLift2.setTargetPosition(lastPos2);
                armLift1.setPower(0.3);
                armLift2.setPower(0.3);
                armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Mecanum drive controls
            if (Math.abs(y) < deadzone && Math.abs(x) < deadzone && Math.abs(rx) < deadzone) {
                frontLeftTargetPower = 0;
                backLeftTargetPower = 0;
                frontRightTargetPower = 0;
                backRightTargetPower = 0;
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } else {
                double denominator = max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftTargetPower = 1.1 * (y + x + rx) / denominator;
                backLeftTargetPower = (y - x + rx) / denominator;
                frontRightTargetPower = 1.1 * (y - x - rx) / denominator;
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