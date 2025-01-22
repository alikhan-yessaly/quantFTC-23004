package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
@TeleOp(name = "EnhancedMergedTeleOp")
public class EnchancedMergedTeleOp extends LinearOpMode {
    // Tuning parameters
    public static double HEADING_kP = 0.04, HEADING_kD = 0.0035;
    public static double TURN_SPEED = 360;

    // Deadzone to prevent unwanted movement
    public static double DEADZONE = 0.1;

    private Follower follower;
    private MecanumDrive drive;
    private PDController headingController;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        Motor leftRear = new Motor(hardwareMap, "leftBack");
        Motor rightRear = new Motor(hardwareMap, "rightBack");
        Motor leftFront = new Motor(hardwareMap, "leftFront");
        Motor rightFront = new Motor(hardwareMap, "rightFront");

        // Set up motor inversion (adjust as needed for correct movement)
        leftFront.setInverted(false); // Adjust based on your setup
        rightFront.setInverted(true); // Adjust based on your setup
        leftRear.setInverted(false);  // Adjust based on your setup
        rightRear.setInverted(true);  // Adjust based on your setup

        // Mecanum drive setup
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);

        // Initialize Follower
        try {
            follower = new Follower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0)); // Initial robot pose
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize Follower. Check 'odo' connection.");
            telemetry.update();
            stop();
        }

        // Initialize PD controller for heading control
        headingController = new PDController(HEADING_kP, HEADING_kD);

        waitForStart();
        if (follower != null) follower.resetIMU();

        while (opModeIsActive()) {
            // Update follower for real-time localization
            follower.update();

            // Get current pose
            Pose currentPose = follower.getPose();
            double currentHeading = currentPose.getHeading(); // Robot orientation in radians

            // Get joystick inputs
            double forwardInput = -gamepad1.left_stick_y; // Forward/backward
            double strafeInput = -gamepad1.left_stick_x;  // Strafing
            double turnInput = -gamepad1.right_stick_x;  // Turning

            // Apply deadzone to joystick inputs
            forwardInput = Math.abs(forwardInput) > DEADZONE ? forwardInput : 0;
            strafeInput = Math.abs(strafeInput) > DEADZONE ? strafeInput : 0;
            turnInput = Math.abs(turnInput) > DEADZONE ? turnInput : 0;

            // Adjust heading target with turn input
            double headingTarget = currentHeading + turnInput * TURN_SPEED * 0.02; // Delta time ~20ms
            double turnPower = headingController.calculate(currentHeading, headingTarget);

            // Drive the robot using Follower-based heading
            drive.driveFieldCentric(strafeInput, forwardInput, turnPower, Math.toDegrees(currentHeading));

            // Telemetry diagnostics
            telemetry.addData("X Position", currentPose.getX());
            telemetry.addData("Y Position", currentPose.getY());
            telemetry.addData("Heading (Degrees)", Math.toDegrees(currentHeading));
            telemetry.addData("Forward Input", forwardInput);
            telemetry.addData("Strafe Input", strafeInput);
            telemetry.addData("Turn Input", turnInput);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
    }
}
