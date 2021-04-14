package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("ALL")
@Autonomous(name="let me jop", group="auto")
//@Disabled

public class DoALittleJopping extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        DROP,   // Drop wobble 1
        TURN,         // Then we want to do a point turn
        SHOOT,   // Shoot all 3 rings
        TRAJ_2,         // Move to 2nd wobble
        GRAB,              // Grab 2nd wobble
        TRAJ_3,             // Move to drop zone
        DROP_2,         // Drop 2nd wobble
        PARK,           // We park
        IDLE            // Our bot will enter the IDLE state when done
    }
    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-62, -45, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory wobbleDropOne = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, -52), Math.toRadians(319))

                .build();

        double turnAngle = Math.toRadians(56);

        Pose2d newLastPose = wobbleDropOne.end().plus(new Pose2d(0, 0, turnAngle));

        Trajectory wobbleGrab = drive.trajectoryBuilder(newLastPose)
                .splineTo(new Vector2d(-42, -40), Math.toRadians(88))
                .build();

        Trajectory wobbleDropTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .splineTo(new Vector2d(0, -52), Math.toRadians(319))
                .build();

        Trajectory park = drive.trajectoryBuilder(wobbleDropTwo.end())
                .splineTo(new Vector2d(10, -10), Math.toRadians(0))
                .build();




        waitForStart();

        drive.claw.setPosition(0);

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(wobbleDropOne);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:

                    if(!drive.isBusy()){
                        currentState = State.DROP;


                    }
                    break;

                case DROP:

                    if (!drive.isBusy()){
                        currentState = State.TURN;

                        drive.turnAsync(turnAngle);
                    }
                    break;

                case TURN:

                    if (!drive.isBusy()){
                        currentState = State.SHOOT;
                        drive.waitTimer.reset();
                        drive.shoot();


                    }
                    break;

                case SHOOT:

                    if (drive.waitTimer.milliseconds() > 1900){
                        currentState = State.TRAJ_2;

                        drive.followTrajectoryAsync(wobbleGrab);
                    }
                    break;

                case TRAJ_2:

                    if (!drive.isBusy()){
                        currentState = State.DROP_2;

                        drive.followTrajectoryAsync(wobbleDropTwo);
                    }
                    break;

                case DROP_2:

                    if (!drive.isBusy()){
                        currentState = State.PARK;

                        drive.followTrajectoryAsync(park);
                    }
                    break;

                case PARK:

                    if (!drive.isBusy()){
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

    }
}

