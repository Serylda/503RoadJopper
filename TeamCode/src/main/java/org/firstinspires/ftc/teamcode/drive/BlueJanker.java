package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision;

@SuppressWarnings("ALL")
@Autonomous(name="jank bluemo", group="auto")
//@Disabled

public class BlueJanker extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        DROP,   // Drop wobble 1
        TURN,         // Then we want to do a point turn
        SHOOT,   // Shoot all 3 rings
        TRAJ_2,
        BACK, // Move to 2nd wobble
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
        BACK,
        GRAB, // grab wobble
        TRAJ_4, // go to wobble drop
        DROP_2, // drop wobble
        PARK, // park
        IDLE, // idle
        TRAJ_5,
        TURN_2,
        TURN_3,
        TURN_4
    }

    enum State1 {
        TRAJECTORY_1,
        DROP,
        BACK,
        SHOOT,
        SHOOTPOS,
        TURN,
        TURNBACK,
        BACK_2,
        WOBBLE,
        GRAB,
        TRAJ_3,
        DROP_2,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;
    State1 currentState1 = State1.IDLE;
    State4 currentState4 = State4.IDLE;

    Pose2d startPose = new Pose2d(-62, 45, Math.toRadians(0));


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

        Trajectory wobbleDropOne = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 53, Math.toRadians(20)))
                .build();

        double turnAngle = Math.toRadians(-38);
        Pose2d newLastPose = wobbleDropOne.end().plus(new Pose2d(0, 0, turnAngle));

        /*
        Trajectory backUp = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-50, 53)) //ctor2d(-43.5, -52), Math.toRadians(176))
                .build();


        Trajectory wobbleGrab = drive.trajectoryBuilder(backUp.end())
                //.forward(18)
                .lineToLinearHeading(new Pose2d(-52, 37, Math.toRadians(-90)))
                .build();

        Trajectory wobbleDropTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .lineToLinearHeading(new Pose2d(-3, 53, Math.toRadians(0)))
                .build();
        */
        Trajectory strafe = drive.trajectoryBuilder(newLastPose)
                .strafeRight(20)
                //.splineTo(new Vector2d(-8, -30), Math.toRadians(319))
                //.splineTo(new Vector2d(15, 0), Math.toRadians(0))
                //.splineTo(new Vector2d(15, -30), Math.toRadians(0))
                .build();



        Trajectory park = drive.trajectoryBuilder(strafe.end())
                .forward(20)
                .build();



        Trajectory case4WobbleOne = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(52, 53, Math.toRadians(10)))
                .build();


        double case4turn1 = Math.toRadians(-10);

        Pose2d case4Turn1Pose = case4WobbleOne.end().plus(new Pose2d(0, 0, case4turn1));

        /*Trajectory case4back1 = drive.trajectoryBuilder(case4WobbleOne.end())
                .back(40)
                .build();*/

        /*
        Trajectory beruh = drive.trajectoryBuilder(case4Turn1Pose)
                .lineToLinearHeading(new Pose2d(0, -52, Math.toRadians(0))) //0,-52, 319/0
                .build();


        Trajectory case4WobbleTwo = drive.trajectoryBuilder(wobbleGrab.end())
                .lineToLinearHeading(new Pose2d(45, -52, Math.toRadians(330)))
                .build();

        /*Trajectory case4back2 = drive.trajectoryBuilder(case4WobbleTwo.end())
                .back(40)
                .build();

        double case4turn2 = Math.toRadians(30);

        Pose2d case4Turn2Pose = wobbleDropOne.end().plus(new Pose2d(0, 0, case4turn2));
        */

        Trajectory case4park = drive.trajectoryBuilder(case4Turn1Pose)
                .lineTo(new Vector2d(10, 30))
                .build();




        Trajectory drop1c1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(22, 43, Math.toRadians(-45)))
                .build();

        Trajectory back1c1 = drive.trajectoryBuilder(drop1c1.end())
                .lineToConstantHeading(new Vector2d(-10, 41))
                .build();

        Trajectory shootPos = drive.trajectoryBuilder(back1c1.end())
                .lineToLinearHeading(new Pose2d(0, 54, Math.toRadians(-16.5)))
                .build();

        /*
        double shootTurnc1 = Math.toRadians(17.5);

        Pose2d shootTurnPosec1 = case4WobbleOne.end().plus(new Pose2d(0, 0, shootTurnc1));

        */
        /*
        double shootTurnBackc1 = Math.toRadians(17);

        Pose2d shootTurnPose2c1 = case4WobbleOne.end().plus(new Pose2d(0, 0, shootTurnBackc1));

        Trajectory back2c1 = drive.trajectoryBuilder(shootPos.end())
                .lineToLinearHeading(new Pose2d(-42, -52, Math.toRadians(90))) //ctor2d(-43.5, -52), Math.toRadians(176))
                .build();

        Trajectory wobbleGrabc1 = drive.trajectoryBuilder(back2c1.end())
                .lineTo(new Vector2d(-42.35, -35.75))
                .build();

        Trajectory drop2c1 = drive.trajectoryBuilder(wobbleGrabc1.end())
                .lineToLinearHeading(new Pose2d(18, -38, Math.toRadians(5)))
                .build();
        */

        Trajectory parkc1 = drive.trajectoryBuilder(shootPos.end())
                .strafeLeft(20)
                .build();


        while (!isStarted()) {
            drive.claw.setPosition(0);
            //drive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
            //drive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            ringCount = vision.ringCount('r');
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
                    currentState = State.TRAJECTORY_1;
                    drive.claw.setPosition(0);
                    drive.followTrajectoryAsync(wobbleDropOne);

                    while (opModeIsActive() && !isStopRequested()) {
                        switch (currentState) {
                            case TRAJECTORY_1:

                                if (!drive.isBusy()) {
                                    currentState = State.DROP;
                                    drive.waitTimer.reset();
                                    drive.wobbleDrop();

                                }
                                break;

                            case DROP:

                                if (!drive.isBusy()) {
                                    currentState = State.TURN;

                                    drive.turnAsync(turnAngle);
                                }
                                break;

                            case TURN:

                                if (!drive.isBusy()) {
                                    currentState = State.DROP_2;
                                    drive.waitTimer.reset();
                                    drive.shoot();


                                }
                                break;
                            /*
                            case SHOOT:

                                if (drive.waitTimer.milliseconds() > 1900) {
                                    currentState = State.TRAJ_2;

                                    drive.followTrajectoryAsync(backUp);
                                }
                                break;

                            case TRAJ_2:

                                if (!drive.isBusy()) {
                                    currentState = State.BACK;
                                    drive.followTrajectoryAsync(wobbleGrab);
                                }
                                break;


                            case BACK:
                                if (!drive.isBusy()) {
                                    currentState = State.GRAB;
                                    drive.waitTimer.reset();
                                    drive.wobbleGrab();
                                }
                                break;


                            case GRAB:
                                if (!drive.isBusy()) {
                                    currentState = State.TRAJ_3;


                                    drive.followTrajectoryAsync(wobbleDropTwo);
                                }

                            case TRAJ_3:
                                if (!drive.isBusy()) {
                                    currentState = State.DROP_2;
                                    drive.waitTimer.reset();
                                    drive.wobbleDrop2();


                                }

                             */
                            case DROP_2:

                                if (!drive.isBusy()) {
                                    drive.waitTimer.reset();
                                    //drive.wobbleGrab();
                                    currentState = State.PARK;

                                    drive.followTrajectoryAsync(strafe);
                                    drive.followTrajectoryAsync(park);
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


                case 1:
                    currentState1 = State1.TRAJECTORY_1;
                    drive.claw.setPosition(0);
                    drive.followTrajectoryAsync(drop1c1);

                    while (opModeIsActive() && !isStopRequested()) {
                        switch (currentState1) {
                            case TRAJECTORY_1:

                                if (!drive.isBusy()) {
                                    currentState1 = State1.DROP;
                                    drive.waitTimer.reset();
                                    drive.wobbleDrop();

                                }
                                break;

                            case DROP:

                                if (!drive.isBusy()) {
                                    currentState1 = State1.BACK;

                                    drive.followTrajectoryAsync(back1c1);
                                }
                                break;

                            case BACK:

                                if (!drive.isBusy()) {
                                    currentState1 = State1.SHOOTPOS;
                                    drive.followTrajectoryAsync(shootPos);

                                }
                                break;

                            case SHOOTPOS:
                                /*
                                if (!drive.isBusy()) {
                                    currentState1 = State1.TURN;

                                    drive.turnAsync(shootTurnc1);

                                }

                                break;

                            case TURN:*/
                                if(!drive.isBusy()){
                                    currentState1 = State1.DROP_2;
                                    drive.waitTimer.reset();
                                    drive.shoot();

                                }
                                break;

                                /*
                            case SHOOT:
                                if(!drive.isBusy()){
                                    currentState1 = State1.TURNBACK;

                                    drive.turnAsync(shootTurnBackc1);
                                }
                                break;

                            case TURNBACK:
                                if(!drive.isBusy()){
                                    currentState1 = State1.BACK_2;

                                    drive.followTrajectoryAsync(back2c1);



                                }
                                break;



                            case BACK_2:

                                if (!drive.isBusy()) {
                                    currentState1 = State1.WOBBLE;

                                    drive.followTrajectoryAsync(wobbleGrabc1);

                                }
                                break;

                            case WOBBLE:

                                if(!drive.isBusy()){
                                    currentState1 = State1.GRAB;

                                    drive.waitTimer.reset();
                                    drive.wobbleGrab();

                                }

                            case GRAB:
                                if (!drive.isBusy()) {
                                    currentState1 = State1.TRAJ_3;


                                    drive.followTrajectoryAsync(drop2c1);
                                }

                            case TRAJ_3:
                                if (!drive.isBusy()) {
                                    currentState1 = State1.DROP_2;
                                    drive.waitTimer.reset();
                                    drive.wobbleDrop2();


                                }


                                 */
                            case DROP_2:

                                if (!drive.isBusy()) {
                                    currentState1 = State1.PARK;
                                    drive.followTrajectoryAsync(parkc1);
                                }
                                break;

                            case PARK:

                                if (!drive.isBusy()) {
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

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TURN;
                                    drive.turnAsync(turnAngle);

                                }
                                break;

                            case TURN:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.SHOOT;
                                    drive.waitTimer.reset();
                                    drive.shoot();

                                }
                                break;

                            case SHOOT:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TRAJ_2;

                                    drive.followTrajectoryAsync(case4WobbleOne);

                                }
                                break;

                            case TRAJ_2:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TURN_3;
                                    drive.waitTimer.reset();
                                    drive.wobbleDrop();
                                }
                                break;

                            case TURN_3:
                                if (!drive.isBusy()) {
                                    currentState4 = State4.DROP_2;
                                    drive.turnAsync(case4turn1);

                                }

                                /*
                            case DROP:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TURN_2;

                                    drive.followTrajectoryAsync(beruh);
                                    telemetry.addLine("yo what's popping");
                                    telemetry.update();


                                }
                                break;

                            case TURN_2:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TRAJ_3;
                                    drive.turnAsync(turnAngle);
                                    telemetry.addLine("I hate my wife");
                                    telemetry.update();


                                }

                            case TRAJ_3:
                                if (!drive.isBusy()) {
                                    currentState4 = State4.BACK;

                                    drive.followTrajectoryAsync(backUp);
                                }
                                break;

                            case BACK:
                                if (!drive.isBusy()) {
                                    currentState4 = State4.TRAJ_4;

                                    drive.followTrajectoryAsync(wobbleGrab);
                                }
                                break;


                            case TRAJ_4:
                                if (!drive.isBusy()) {
                                    currentState4 = State4.GRAB;
                                    drive.waitTimer.reset();
                                    drive.wobbleGrab();
                                }

                            case GRAB:
                                if (!drive.isBusy()) {
                                    currentState4 = State4.TRAJ_5;

                                    drive.followTrajectoryAsync(case4WobbleTwo);

                                }
                            case TRAJ_5:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.TURN_4;

                                    drive.waitTimer.reset();
                                    drive.wobbleDrop2();
                                }
                                break;

                            case TURN_4:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.DROP_2;
                                    drive.turnAsync(case4turn2);


                                }


                                 */
                            case DROP_2:

                                if (!drive.isBusy()) {
                                    currentState4 = State4.PARK;
                                    drive.followTrajectoryAsync(case4park);
                                }
                                break;


                            case PARK:

                                if (!drive.isBusy()) {
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

