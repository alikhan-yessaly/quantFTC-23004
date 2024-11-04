package org.firstinspires.ftc.teamcode.notcompetition.AUTO;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;
        private Servo liftUpL;
        private Servo liftUpR;


        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "lift1" );
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift1.setDirection(DcMotorSimple.Direction.REVERSE);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftUpR = hardwareMap.get(Servo.class, "liftUp");
            liftUpL = hardwareMap.get(Servo.class, "liftUpL");
        }

        public class LiftExt implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(0.8);
                    lift2.setPower(0.8);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                packet.put("lift2Pos", pos2);
                if (pos1 < 3000.0 && pos2 < 3000.0) { // Check both motors
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftExt() {
            return new LiftExt();
        }

        public class LiftRet implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift1.setPower(-0.8);
                    lift2.setPower(-0.8);
                    initialized = true;
                }

                double pos1 = lift1.getCurrentPosition();
                packet.put("lift1Pos", pos1);
                double pos2 = lift2.getCurrentPosition();
                packet.put("lift2Pos", pos2);

                if (pos1 > 100.0 && pos2 > 100.0) {
                    return true;
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftRet(){
            return new LiftRet();
        }
        public class LiftUp implements Action {
            private boolean initialized = false;
            public double targetPos = 0.8;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftUpL.setPosition(0.5);
                    liftUpR.setPosition(0.5);
                    initialized = true;
                }
                double pos1 = liftUpL.getPosition();
                double pos2 = liftUpR.getPosition();
                packet.put("liftUpL", pos1);
                packet.put("liftUpR", pos2);
                return pos1 >= targetPos && pos2 >= targetPos;

            }
            public Action liftUp(){
                return new LiftUp();
            }
        public class LiftDown implements Action {
            private boolean initialized = false;
            public double targetPos = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftUpL.setPosition(0.5);
                    liftUpR.setPosition(0.5);
                    initialized = true;
                }
                double pos1 = liftUpL.getPosition();
                double pos2 = liftUpR.getPosition();
                packet.put("liftUpL", pos1);
                packet.put("liftUpR", pos2);
                return pos1 <= targetPos && pos2 <= targetPos;
            }
            }
            public Action liftDown(){
                return new LiftDown();
            }


        }
    }

    public class Arm{
        private Servo arm0;
        private Servo arm1;
        public Arm(HardwareMap hardwareMap){
            arm0 = hardwareMap.get(Servo.class, "arm0");
            arm1 = hardwareMap.get(Servo.class, "arm1");
        }


    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.45);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftExt(),
                        claw.openClaw(),
                        lift.liftRet(),
                        trajectoryActionCloseOut
                )
        );
    }
}

