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
        TRAJECTORY_1, //drive to shooting zone
        TURN, // turn towards goal
        SHOOT, // shoot 3 rings
        TRAJ_2, // go to wobble drop
        DROP, // drop wobble
        TRAJ_3, // go to wobble grab
        GRAB, // grab wobble
        TRAJ_4, // go to wobble drop
        DROP_2, // drop wobble
        PARK, // park
        IDLE, // idle
        TRAJ_5,
        TURN_2
    }

    enum State1{
        TRAJECTORY_1,
        DROP,
        BACK,
        SHOOT,
        TRAJ_2,
        GRAB,
        TRAJ_3,
        DROP_2,
        IDLE
    }

    State currentState = State.IDLE;
    State1 currentState1 = State1.IDLE;
    State4 currentState4 = State4.IDLE;

    Pose2d startPose = new Pose2d(-62, -45, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Vision vision = new Vision(this);
        drive.claw.setPosition(0);

        drive.setPoseEstimate(startPose);

        Trajectory wobbleDropOne = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, -54), Math.toRadians(319))

                .build();

        double turnAngle = Math.toRadians(56.5);

        Pose2d newLastPose = wobbleDropOne.end().plus(new Pose2d(0, 0, turnAngle));



        Trajectory backUp = drive.trajectoryBuilder(newLastPose)
                .splineTo(new Vector2d(-43.5, -52), Math.toRadians(0))
                .build();


        Trajectory wobbleGrab = drive.trajectoryBuilder(backUp.end())
                .splineTo(new Vector2d(-41, -34), Math.toRadians(94))
                .build();

        Trajectory wobbleDropTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .splineTo(new Vector2d(-5, -54), Math.toRadians(350))
                .build();

        Trajectory strafe = drive.trajectoryBuilder(wobbleDropTwo.end())
                .strafeLeft(25)
                //.splineTo(new Vector2d(-8, -30), Math.toRadians(319))
                //.splineTo(new Vector2d(15, 0), Math.toRadians(0))
                //.splineTo(new Vector2d(15, -30), Math.toRadians(0))
                .build();


        Trajectory park = drive.trajectoryBuilder(strafe.end())
                .forward(10)
                .build();

        Trajectory case4WobbleOne = drive.trajectoryBuilder(newLastPose)
                .splineTo(new Vector2d(52, -52), Math.toRadians(-41))
                .build();

        Trajectory case4back1 = drive.trajectoryBuilder(case4WobbleOne.end())
                .back(40)
                .build();

        Trajectory beruh = drive.trajectoryBuilder(case4back1.end())
                .splineTo(new Vector2d(0, -52), Math.toRadians(319)) //0,-52, 319/0
                .build();



        Trajectory case4WobbleTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .splineTo(new Vector2d(45, -52), Math.toRadians(-41))
                .build();

        Trajectory case4back2 = drive.trajectoryBuilder(case4WobbleTwo.end())
                .back(40)
                .build();


        Trajectory case4park = drive.trajectoryBuilder(case4back2.end())
                .splineTo(new Vector2d( 10, -30), Math.toRadians(0))
                .build();



        Trajectory case1Traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(18.7, -38.4), 0)
                .build();

        Trajectory case1Back = drive.trajectoryBuilder(case1Traj1.end())
                .splineTo(new Vector2d(-3.2, -34.6), 348)
                .build();

        Trajectory case1Traj2 = drive.trajectoryBuilder(case1Back.end())
                .splineTo(new Vector2d(-44, -40), 85.6)
                .build();

        Trajectory case1Traj3 = drive.trajectoryBuilder(case1Traj2.end())
                .splineTo(new Vector2d(14, -38), 0)
                .build();









        waitForStart();

        drive.claw.setPosition(0);
        drive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
        drive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        int ringCount = 4;
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
                                drive.waitTimer.reset();
                                //drive.wobbleGrab();
                                currentState = State.PARK;

                                drive.followTrajectoryAsync(strafe);
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
                currentState1 = State1.TRAJECTORY_1;
                drive.claw.setPosition(0);
                drive.followTrajectoryAsync(case1Traj1);

                while (opModeIsActive() && !isStopRequested()) {
                    switch (currentState1) {
                        case TRAJECTORY_1:

                            if(!drive.isBusy()){
                                currentState1 = State1.DROP;
                                drive.waitTimer.reset();
                                drive.wobbleDrop();

                            }
                            break;

                        case DROP:

                            if (!drive.isBusy()){
                                currentState1 = State1.BACK;

                                drive.followTrajectoryAsync(case1Back);
                            }
                            break;

                        case BACK:

                            if (!drive.isBusy()){
                                currentState1 = State1.SHOOT;
                                drive.waitTimer.reset();
                                drive.shoot();


                            }
                            break;

                        case SHOOT:

                            if (!drive.isBusy()){
                                currentState1 = State1.TRAJ_2;

                                drive.followTrajectoryAsync(case1Traj2);
                            }
                            break;

                        case TRAJ_2:

                            if (!drive.isBusy()){
                                currentState1 = State1.GRAB;
                                drive.waitTimer.reset();
                                drive.wobbleGrab();
                            }
                            break;

                        case GRAB:
                            if(!drive.isBusy()){
                                currentState1 = State1.TRAJ_3;



                                drive.followTrajectoryAsync(case1Traj3);
                            }

                        case TRAJ_3:
                            if(!drive.isBusy()){
                                currentState1 = State1.DROP_2;
                                drive.waitTimer.reset();
                                drive.wobbleDrop2();


                            }
                        case DROP_2:

                            if (!drive.isBusy()){
                                currentState1 = State1.IDLE;


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

            case 4:
                currentState4 = State4.TRAJECTORY_1;
                drive.claw.setPosition(0);
                drive.followTrajectoryAsync(wobbleDropOne);

                while (opModeIsActive() && !isStopRequested()) {
                    switch (currentState4) {
                        case TRAJECTORY_1:

                            if(!drive.isBusy()){
                                currentState4 = State4.TURN;
                                drive.turnAsync(turnAngle);

                            }
                            break;

                        case TURN:

                            if (!drive.isBusy()){
                                currentState4 = State4.SHOOT;
                                drive.waitTimer.reset();
                                drive.shoot();

                            }
                            break;

                        case SHOOT:

                            if (!drive.isBusy()){
                                currentState4 = State4.TRAJ_2;

                                drive.followTrajectoryAsync(case4WobbleOne);

                            }
                            break;

                        case TRAJ_2:

                            if (!drive.isBusy()){
                                currentState4 = State4.DROP;
                                drive.waitTimer.reset();
                                drive.wobbleDrop();
                            }
                            break;

                        case DROP:

                            if (!drive.isBusy()){
                                currentState4 = State4.TURN_2;
                                drive.followTrajectoryAsync(case4back1);
                                drive.followTrajectoryAsync(beruh);
                                telemetry.addLine("yo what's popping");
                                telemetry.update();



                            }
                            break;

                        case TURN_2:

                            if(!drive.isBusy()){
                                currentState4 = State4.TRAJ_3;
                                drive.turnAsync(turnAngle);
                                telemetry.addLine("I hate my wife");
                                telemetry.update();


                            }

                        case TRAJ_3:
                            if(!drive.isBusy()){
                                currentState4 = State4.TRAJ_4;

                                drive.followTrajectoryAsync(backUp);
                                drive.followTrajectoryAsync(wobbleGrab);
                            }
                            break;

                        case TRAJ_4:
                            if(!drive.isBusy()){
                                currentState4 = State4.GRAB;
                                drive.waitTimer.reset();
                                drive.wobbleGrab();
                            }

                        case GRAB:
                            if(!drive.isBusy()){
                                currentState4 = State4.TRAJ_5;

                                drive.followTrajectoryAsync(case4WobbleTwo);

                            }
                        case TRAJ_5:

                            if (!drive.isBusy()){
                                currentState4 = State4.DROP_2;

                                drive.waitTimer.reset();
                                drive.wobbleDrop2();
                            }
                            break;

                        case DROP_2:

                            if (!drive.isBusy()){
                                currentState4 = State4.PARK;
                                drive.followTrajectoryAsync(case4back2);
                                drive.followTrajectoryAsync(case4park);
                            }
                            break;

                        case PARK:

                            if (!drive.isBusy()){
                                currentState4 = State4.IDLE;


                        }
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

