package org.firstinspires.ftc.teamcode.notcompetition.TELEOP;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.Wrist;
import org.firstinspires.ftc.teamcode.utils.Claw;
import org.firstinspires.ftc.teamcode.utils.Arm;
import org.firstinspires.ftc.teamcode.utils.LiftUp;

import java.io.FileWriter;
import java.io.IOException;
import java.io.File;
import java.util.Locale;

@TeleOp(name = "QuantumDriveWithTelemetry")
public class QuantumDriveWithTelemetry extends QuantumDrive {
    private static final String TAG = "QuantumDriveWithTelemetry";
    private Wrist wrist;
    private Claw claw;
    private Arm arm;
    private LiftUp liftUp;

    private boolean yPressedPreviously = false;
    private int positionCounter = 0;

    @Override
    public void runOpMode() {
        // Initialize servos and other components using QuantumDrive setup
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        liftUp = new LiftUp(hardwareMap);

        super.runOpMode();

        while (opModeIsActive()) {
            // Display servo positions in telemetry
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Arm Position 0", arm.getArm0Position());
            telemetry.addData("Arm Position 1", arm.getArm1Position());
            telemetry.addData("Lift Position", liftUp.getPosition());
            telemetry.update();

            // Check if the Y button is pressed and hasn't been recorded
            if (gamepad1.y && !yPressedPreviously) {
                recordServoPositions();
                telemetry.addData("Status", "Position " + positionCounter + " is recorded.");
                telemetry.update();
                positionCounter++;  // Increment the position counter after each recording
                yPressedPreviously = true;
            } else if (!gamepad1.y) {
                yPressedPreviously = false;
            }
        }
    }

    private void recordServoPositions() {
        // Prepare data to write
        String data = String.format(
                Locale.US,
                "Position %d - Wrist: %.2f, Claw: %.2f, Arm0: %.2f, Arm1: %.2f, Lift: %.2f%n",
                positionCounter, wrist.getPosition(), claw.getPosition(),
                arm.getArm0Position(), arm.getArm1Position(),
                liftUp.getPosition()
        );

        // Define the file path in the external storage directory
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/servo_positions.txt");

        // Ensure the directory exists
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            boolean dirCreated = parentDir.mkdirs();
            if (!dirCreated) {
                telemetry.addData("Error", "Failed to create directory for servo positions.");
                telemetry.update();
                return;
            }
        }

        // Try writing to the file and check if it’s successful
        try (FileWriter writer = new FileWriter(file, true)) {
            writer.write(data);
            telemetry.addData("Status", "Position " + positionCounter + " successfully recorded.");
            Log.i(TAG, "Servo positions recorded: " + data.trim());
        } catch (IOException e) {
            telemetry.addData("Error", "Could not write to file.");
            Log.e(TAG, "Error writing to file: " + file.getPath(), e);
        }
        telemetry.update();
    }
}