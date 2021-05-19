package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

@SuppressWarnings("ALL")
@Autonomous(name="blue support", group="auto")
//@Disabled

public class BlueSup extends LinearOpMode {

    enum State {
        STRAFEOUT,
        MOVEFORWARD,
        SHOOT,
        PARK,
        IDLE
    }

    ElapsedTime time = new ElapsedTime();

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-62, 21, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vision vision = new Vision(this);
        int ringCount = vision.ringCount('b');
        telemetry.addData("Ring Count: ", ringCount);
        telemetry.update();
        drive.claw.setPosition(0);

        drive.setPoseEstimate(startPose);

        Trajectory strafeOut = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-62, 15))
                .build();

        Trajectory moveForward = drive.trajectoryBuilder(strafeOut.end())
                .lineToLinearHeading(new Pose2d(0, 15,Math.toRadians(5)))
                .build();

        double turn1 = Math.toRadians(-5);
        Pose2d t1pose = moveForward.end().plus(new Pose2d(0, 0, turn1));

        double turn2 = Math.toRadians(-5);
        Pose2d t2pose = t1pose.plus(new Pose2d(0, 0, turn2));

        Trajectory park = drive.trajectoryBuilder(t2pose)
                .forward(20)
                .build();


        while (!isStarted()) {
            drive.claw.setPosition(0);
            //drive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
            //drive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            ringCount = vision.ringCount('b');
            telemetry.addData("Ring Count: ", ringCount);
            telemetry.update();
        }


            waitForStart();

            drive.claw.setPosition(0);
            //drive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
            //drive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);

            if (isStopRequested()) return;

            switch (ringCount) {        //ringCount

                case 0:
                    currentState = State.STRAFEOUT;
                    drive.claw.setPosition(0);
                    drive.followTrajectoryAsync(strafeOut);

                    while (opModeIsActive() && !isStopRequested()) {
                        switch (currentState) {
                            case STRAFEOUT:

                                if (!drive.isBusy()) {
                                    currentState = State.MOVEFORWARD;
                                    drive.waitTimer.reset();
                                    drive.followTrajectoryAsync(moveForward);

                                }
                                break;

                            case MOVEFORWARD:

                                if (!drive.isBusy()) {
                                    currentState = State.SHOOT;

                                }
                                break;

                            case SHOOT:

                                if (!drive.isBusy()) {
                                    currentState = State.PARK;
                                    drive.waitTimer.reset();

                                    time.reset();

                                    while (time.milliseconds() < 5000) {
                                        drive.FlyWheel1.setPower(0.8);
                                        drive.FlyWheel2.setPower(0.8);

                                        if (time.milliseconds() >= 500 && time.milliseconds() < 650){
                                            drive.ringHopper.setPosition(0.9);
                                        }
                                        if (time.milliseconds() >= 650 && time.milliseconds() < 1000){
                                            drive.ringHopper.setPosition(0.5);
                                        }

                                    }
                                }
                                break;

                            case PARK:

                                if (!drive.isBusy()) {
                                    currentState = State.IDLE;


                                }
                                break;
                            case IDLE:

                                break;
                        }

                        drive.update();
                        Pose2d poseEstimate = drive.getPoseEstimate();


                        // Print pose to telemetry
                        telemetry.addData("x", poseEstimate.getX());
                        telemetry.addData("y", poseEstimate.getY());
                        telemetry.addData("heading", poseEstimate.getHeading());
                        telemetry.update();


                    }
                    break;
                    }
            }
        }
