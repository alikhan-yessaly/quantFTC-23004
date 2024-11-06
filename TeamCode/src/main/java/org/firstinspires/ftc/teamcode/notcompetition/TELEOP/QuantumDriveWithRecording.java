package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.LiftUp;
import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.Wrist;
import org.firstinspires.ftc.teamcode.utils.Claw;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "QuantumDriveWithRecording")
public class QuantumDriveWithRecording extends LinearOpMode {
    private static final String TAG = "QuantumDriveWithRecording";

    private LiftUp liftUp;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;
    private DcMotor armLift1, armLift2;

    private int positionCounter = 1; // Counter for recorded positions
    private boolean xPressedPreviously = false;
    private boolean yPressedPreviously = false;

    @Override
    public void runOpMode() {
        // Initialize drivetrain motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        armLift1 = hardwareMap.dcMotor.get("lift1");
        armLift2 = hardwareMap.dcMotor.get("lift2");

        int lastPos1 = 0;
        int lastPos2 = 0;

        // Initialize lift and other mechanism components
        liftUp = new LiftUp(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);

        armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            boolean liftUpTrigger = gamepad1.right_bumper;
            boolean liftDownTrigger = gamepad1.right_trigger > 0.1;
            boolean liftOut = gamepad1.left_bumper;
            boolean liftIn = gamepad1.left_trigger > 0.1;

            // Arm control with D-Pad
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

            // Wrist cycling positions with X button
            if (gamepad1.x && !xPressedPreviously) {
                wrist.cyclePosition();
                xPressedPreviously = true;
            } else if (!gamepad1.x) {
                xPressedPreviously = false;
            }

            // LiftUp control
            if (liftUpTrigger) {
                liftUp.raise(0.01);
            } else if (liftDownTrigger) {
                liftUp.lower(0.01);
            }

            // Arm lift control
            int armLift1CurrentPosition = armLift1.getCurrentPosition();
            int armLift2CurrentPosition = armLift2.getCurrentPosition();

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


            // Record servo positions when Y is pressed
            if (gamepad1.y && !yPressedPreviously) {
                recordServoPositions();
                telemetry.addData("Recording", "Position " + positionCounter + " recorded");
                positionCounter++; // Increment the counter after each recording
                yPressedPreviously = true;
            } else if (!gamepad1.y) {
                yPressedPreviously = false;
            }

            // Mecanum drive logic for movement
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            telemetry.update();
        }
    }

    private void recordServoPositions() {
        // Get the current position of armLift1
        int armLift1Position = armLift1.getCurrentPosition();

        // Prepare data to write, including position counter and armLift1 position
        String data = String.format(
                Locale.US,
                "Position %d - Wrist: %.2f, Claw: %.2f, Arm0: %.2f, Arm1: %.2f, Lift: %.2f, ArmLift1: %d%n",
                positionCounter,
                wrist.getPosition(), claw.getPosition(),
                arm.getArm0Position(), arm.getArm1Position(),
                liftUp.getPosition(),
                armLift1Position
        );

        // Define the file path
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/servo_positions.txt");

        // Ensure the directory exists
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            if (!parentDir.mkdirs()) {
                telemetry.addData("Error", "Failed to create directory for servo positions.");
                telemetry.update();
                return; // Exit if directory creation failed
            }
        }

        try (FileWriter writer = new FileWriter(file, true)) {
            writer.write(data);
            Log.i(TAG, "Position " + positionCounter + " recorded: " + data.trim());
            telemetry.addData("Recording", "Position " + positionCounter + " recorded with ArmLift1: " + armLift1Position);
        } catch (IOException e) {
            telemetry.addData("Error", "Could not write to file.");
            Log.e(TAG, "Error writing to file: " + file.getPath(), e);
        }
        telemetry.update();
    }
}
