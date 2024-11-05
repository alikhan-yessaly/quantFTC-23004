package org.firstinspires.ftc.teamcode.notcompetition.AUTO;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "TestPedroAuto", group = "Autonomous")
public class TestPedroAuto extends OpMode {

    private Timer pathTimer, opmodeTimer, scanTimer;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(144-(63+72), 12+72, 0);

    private Follower follower;
    private PathChain firstCycle;

    public void buildPaths() {
        firstCycle = follower.pathBuilder()

                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(7.182, 90.359, Point.CARTESIAN),
                                new Point(33.890, 92.603, Point.CARTESIAN),
                                new Point(16.833, 74.873, Point.CARTESIAN),
                                new Point(39.277, 75.546, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathTimer.getElapsedTime() < 500) {
            follower.followPath(firstCycle);
        }
    }

    public void setPathState(int state) {
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
    }

    @Override
    public void init() {
//PhotonCore.start(this.hardwareMap);



        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}
