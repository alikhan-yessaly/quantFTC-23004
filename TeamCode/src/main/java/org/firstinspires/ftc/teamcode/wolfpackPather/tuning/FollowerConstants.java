package org.firstinspires.ftc.teamcode.wolfpackPather.tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;

@Config
public class FollowerConstants {

    // This section is for setting the actual drive vector for the front left wheel
    private static double xMovement = 78.54602;
    private static double yMovement = -55.68387;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Mass of robot in kilograms
    public static double mass = 10.4326;

    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            2,
            0.065,
            0.0000,
            0);

    public static double headingPIDFSwitch = Math.PI/6;

    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            2,
            0.045,
            0.075,
            0);

    // Translational PIDF coefficients
    public static CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.05,
            0,
            0.00275,
            0);

    // Drive PIDF coefficients
    public static CustomPIDFCoefficients drivePIDFCoefficients = new CustomPIDFCoefficients(
            0.001,
            0,
            0.0025,
            0);

    // Centrifugal force to power scaling
    public static double centrifugalScaling = 0;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double zeroPowerAcceleration = -30; // used to be -17.5

    // When the drivetrain is at the end of its current path or path chain and the velocity goes
    // below this value, then end the path. This is in inches/second
    public static double pathEndVelocity = 0.01;

    // When the t-value of the closest point to the robot on the path is greater than this value,
    // then the path is considered at its end.
    public static double pathEndTValue = 0.99;
}
