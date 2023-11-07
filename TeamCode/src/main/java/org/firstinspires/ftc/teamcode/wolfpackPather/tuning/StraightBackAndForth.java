package org.firstinspires.ftc.teamcode.wolfpackPather.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wolfpackPather.follower.Follower;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Point;

@Config
@Autonomous (name = "Straight Back And Forth", group = "Autonomous Pathing Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards, backwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE/2,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setLinearHeadingInterpolation(0,0);
        backwards = new Path(new BezierCurve(new Point(DISTANCE,0, Point.CARTESIAN), new Point(DISTANCE/2,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setLinearHeadingInterpolation(0,0);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("stuff");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryA.addData("Heading error", MathFunctions.getSmallestAngleDifference(0,follower.getPose().getHeading()));
        telemetryA.addData("translational error", follower.asdf());
        telemetryA.addData("isBusy", follower.isBusy());
        telemetryA.addData("heading", follower.getHeadingVector().getMagnitude());
        telemetryA.addData("corrective", follower.getCorrectiveVector().getMagnitude());
        telemetryA.addData("drive", follower.getDriveVector().getMagnitude());
        telemetryA.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
