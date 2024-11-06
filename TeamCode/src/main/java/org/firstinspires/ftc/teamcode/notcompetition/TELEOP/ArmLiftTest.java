package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utils.ArmLift;

@TeleOp(name = "ArmLiftTest")
public class ArmLiftTest extends LinearOpMode {

    private ArmLift armLift;

    @Override
    public void runOpMode() {
        // Initialize the ArmLift class
        armLift = new ArmLift(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Move the arm lift down when pressing D-Pad down
            if (gamepad1.dpad_down) {
                armLift.moveDown();
            }
            // Move the arm lift up when pressing D-Pad up
            else if (gamepad1.dpad_up) {
                armLift.moveUp();
            }

            // Stop the arm lift motors when pressing B
            if (gamepad1.b) {
                armLift.stop();
            }

            // Display the current position and target status
            telemetry.addData("ArmLift Position", armLift.getCurrentPosition());
            telemetry.addData("At Target", armLift.isAtTarget() ? "Yes" : "No");
            telemetry.update();
        }
    }
}
