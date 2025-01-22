package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class YerdaDrive extends LinearOpMode {
    // Declare your motors and other class member variables here
    public static double HEADING_kP = 0.04, HEADING_kD = 0.0035;
    public static double LIFT_kP = 0.01, LIFT_kD = 0.0001;
    public static double EXTENDER_kP = 0.02, EXTENDER_kD = 0.0002;
    public static double LIFT_SPEED = 2200;
    public static double EXTENDER_SPEED = 1000;
    public static double TURN_SPEED = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor liftLeft = new Motor(hardwareMap, "lift1");
        Motor liftRight = new Motor(hardwareMap, "lift2");
        Motor extenderMotor = new Motor(hardwareMap, "lift3");
        Motor intakeMotor = new Motor(hardwareMap, "intake");
        Servo servo1 = hardwareMap.servo.get("claw1");
        Servo servo2 = hardwareMap.servo.get("claw2");
        Servo plane = hardwareMap.servo.get("plane");
        plane.setPosition(1);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.'
        // See the note about this earlier on this page.
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setInverted(true);
        backRight.setInverted(true);
        liftLeft.setInverted(true);

        PDController liftLeftController = new PDController(0, 0);
        PDController liftRightController = new PDController(0, 0);
        PDController headingController = new PDController(0, 0);
        PDController extenderController = new PDController(0, 0);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        
        double liftTargetPosition = 0;
        double headingTarget = 0;
        double extenderTargetPosition = 0;

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        waitForStart();

        imu.resetYaw();
        extenderMotor.resetEncoder();
        liftLeft.resetEncoder();
        liftRight.resetEncoder();

        ElapsedTime runTimer = new ElapsedTime();

        while (opModeIsActive()) {
            driverOp.readButtons();
            toolOp.readButtons();
            double deltaTime = ((double)runTimer.milliseconds() / 1000.0);
            runTimer.reset();

            headingController.setPID(HEADING_kP, 0, HEADING_kD);
            extenderController.setPID(EXTENDER_kP, 0, EXTENDER_kD);
            liftLeftController.setPID(LIFT_kP, 0, LIFT_kD);
            liftRightController.setPID(LIFT_kP, 0, LIFT_kD);
            
            // Main robot control loop code here
            double forwardInput = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double strafeInput = -gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;

            headingTarget += turnInput * TURN_SPEED * deltaTime;

            if (driverOp.wasJustPressed(GamepadKeys.Button.B))
            {
                if ((int)headingTarget % 90 == 0)
                    headingTarget -= 90;
                else
                    headingTarget = Math.floor(headingTarget / 90.0) * 90.0;
            }
            if (driverOp.wasJustPressed(GamepadKeys.Button.X))
            {
                if ((int)headingTarget % 90 == 0)
                    headingTarget += 90;
                else
                    headingTarget = Math.ceil(headingTarget / 90.0) * 90.0;
            }

            if (gamepad1.options) {
                headingTarget = 0;
                imu.resetYaw();
            }
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            while (currentHeading - headingTarget > 180)
                currentHeading -= 360;
            while (currentHeading - headingTarget < -180)
                currentHeading += 360;

            double turnPower = -headingController.calculate(currentHeading, headingTarget);

            drive.driveFieldCentric(strafeInput, forwardInput, turnPower, currentHeading);

            double liftInput = 0;

            if (gamepad1.left_bumper)
                liftInput += 1;
            if (gamepad1.left_trigger > 0.3)
                liftInput -= 1;

            liftTargetPosition += liftInput * LIFT_SPEED * deltaTime;
            if (liftTargetPosition < 0)
                liftTargetPosition = 0;

            double liftLeftPosition = liftLeft.getCurrentPosition();
            double liftRightPosition = liftRight.getCurrentPosition();

            double liftLeftPower = liftLeftController.calculate(liftLeftPosition, liftTargetPosition);
            double liftRightPower = liftRightController.calculate(liftRightPosition, liftTargetPosition);

            telemetry.addData("leftLift", liftLeftPosition);
            telemetry.addData("rightLift", liftRightPosition);
            telemetry.addData("liftTarget", liftTargetPosition);
            telemetry.addData("liftLeftPower", liftLeftPower);
            telemetry.addData("liftRightPower", liftRightPower);

            liftLeft.set(liftLeftPower);
            liftRight.set(liftRightPower);
            
            double intakePower = 0;

            if (gamepad2.right_bumper)
                intakePower += 1;
            if (gamepad2.right_trigger > 0.3)
                intakePower -= 1;
            
            intakeMotor.set(intakePower);

            if (gamepad1.right_bumper) {
                servo1.setPosition(-1);
                servo2.setPosition(1);

            }
            else{
                servo1.setPosition(1);
                servo2.setPosition(-1);//perpendicular 5.906x 3.937y parallel 7.48y 1.969x
            }

            double extenderInput = 0;

            if (gamepad2.left_bumper)
                extenderInput += 1;
            if (gamepad2.left_trigger > 0.3)
                extenderInput -= 1;

            extenderTargetPosition += extenderInput * EXTENDER_SPEED * deltaTime;
            double extenderPosition = extenderMotor.getCurrentPosition();
            extenderMotor.set(extenderController.calculate(extenderPosition, extenderTargetPosition));

            if (extenderTargetPosition < 0)
                extenderTargetPosition = 0;

            if (gamepad2.cross)
                plane.setPosition(-1.0);
            telemetry.update();
        }
    }
}

    // Define the turnRobot method here, outside of runOpMode

