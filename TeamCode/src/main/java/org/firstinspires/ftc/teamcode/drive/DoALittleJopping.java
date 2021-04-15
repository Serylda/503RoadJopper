package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;

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

    enum State4 {
        TRAJECTORY_1,
        TURN,
        SHOOT,
        TRAJ_2,
        DROP,
        TRAJ_3,
        GRAB,
        TRAJ_4,
        DROP_2,
        PARK,
        IDLE
    }

    enum State1{
        TRAJECTORY_1,
        DROP,
        SHOOT,
        TRAJ_2,
        GRAB,
        TRAJ_3,
        DROP_2,
        IDLE
    }

    State currentState = State.IDLE;
    State currentState1 = State.IDLE;
    State currentState4 = State.IDLE;

    Pose2d startPose = new Pose2d(-62, -45, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vision vision = new Vision(this);
        drive.claw.setPosition(0);

        drive.setPoseEstimate(startPose);

        Trajectory wobbleDropOne = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, -52), Math.toRadians(319))

                .build();

        double turnAngle = Math.toRadians(62);

        Pose2d newLastPose = wobbleDropOne.end().plus(new Pose2d(0, 0, turnAngle));

        Trajectory backUp = drive.trajectoryBuilder(newLastPose)
                .splineTo(new Vector2d(-43, -52), Math.toRadians(0))
                .build();


        Trajectory wobbleGrab = drive.trajectoryBuilder(backUp.end())
                .splineTo(new Vector2d(-43, -34), Math.toRadians(90))
                .build();

        Trajectory wobbleDropTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .splineTo(new Vector2d(-10, -50), Math.toRadians(319))
                .build();

        Trajectory park = drive.trajectoryBuilder(wobbleDropTwo.end())
                .splineTo(new Vector2d(0, -30), Math.toRadians(0))
                .build();




        waitForStart();

        drive.claw.setPosition(0);
        drive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
        drive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        int ringCount = vision.ringCount('r');
        telemetry.addData("Ring Count: ", ringCount);
        telemetry.update();

        if (isStopRequested()) return;

        switch (ringCount){

            case 0:
                currentState = State.TRAJECTORY_1;
                drive.claw.setPosition(0);
                drive.followTrajectoryAsync(wobbleDropOne);

                while (opModeIsActive() && !isStopRequested()) {
                    switch (currentState) {
                        case TRAJECTORY_1:

                            if(!drive.isBusy()){
                                currentState = State.DROP;
                                drive.waitTimer.reset();
                                drive.wobbleDrop();

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

                                drive.followTrajectoryAsync(backUp);
                                drive.followTrajectoryAsync(wobbleGrab);
                            }
                            break;

                        case TRAJ_2:

                            if (!drive.isBusy()){
                                currentState = State.GRAB;
                                drive.waitTimer.reset();
                                drive.wobbleGrab();
                            }
                            break;

                        case GRAB:
                            if(!drive.isBusy()){
                                currentState = State.TRAJ_3;



                                drive.followTrajectoryAsync(wobbleDropTwo);
                            }

                        case TRAJ_3:
                            if(!drive.isBusy()){
                                currentState = State.DROP_2;
                                drive.waitTimer.reset();
                                drive.wobbleDrop2();


                            }
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
                break;



            case 1:


                break;

            case 4:

                break;
        }



    }
}

