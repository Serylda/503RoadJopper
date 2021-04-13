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
@Autonomous(name="ROAD JOPPER", group="auto")
//@Disabled

public class RoadJopper extends LinearOpMode {


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62, -45, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory wobbleDropOne = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, -52), Math.toRadians(319))

                .build();

        /*Trajectory shootTurn = drive.trajectoryBuilder(wobbleDropOne.end())
                .splineTo(new Vector2d(0, -52), Math.toRadians(20))
                .build();*/

        Trajectory wobbleGrab = drive.trajectoryBuilder(wobbleDropOne.end())
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
        if(isStopRequested()) return;

        startPose = new Pose2d(-62, -45, Math.toRadians(0));
        ElapsedTime clock = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        //drive.followTrajectory(jmac);
        drive.followTrajectory(wobbleDropOne);

        dropWb(drive);
        drive.turn(Math.toRadians(56));
        jiradShoot(drive);

        if (clock.milliseconds() == 200){

            drive.claw.setPosition(1);
            clock.reset();

        }

        if (clock.milliseconds() == 200){

            drive.Arm.setPower(0);
            clock.reset();

        }

        drive.followTrajectory(wobbleGrab);


        drive.claw.setPosition(0);
        clock.reset();
        if(clock.milliseconds() == 500){

            drive.Arm.setPower(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 400){


            drive.Arm.setPower(0);
            clock.reset();
        }


        drive.followTrajectory(wobbleDropTwo);

        drive.Arm.setPower(-0.5);
        clock.reset();

        if(clock.milliseconds() == 350){
            drive.Arm.setPower(0);
            drive.claw.setPosition(1);
        }



        drive.followTrajectory(park);


        /*Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();*/

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void dropWb(SampleMecanumDrive drive)
    {
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        drive.Arm.setPower(-0.5);

        if (clock.milliseconds() == 350){
            drive.Arm.setPower(0);
            clock.reset();
        }

        if (clock.milliseconds() == 200){

            drive.claw.setPosition(1);
            clock.reset();
        }

        if (clock.milliseconds() == 300){

            drive.Arm.setPower(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 200){
            drive.claw.setPosition(0);
            clock.reset();

        }

        if(clock.milliseconds() == 200){

            drive.Arm.setPower(0);
            clock.reset();

        }

    }

    public void jiradShoot(SampleMecanumDrive drive)
    {
        ElapsedTime clock = new ElapsedTime();
        drive.FlyWheel1.setVelocity(1500 * 13.5 / drive.getVoltage());
        drive.FlyWheel2.setVelocity(1500 * 13.5 / drive.getVoltage());
        drive.ringHopper.setPosition(0.9);
        clock.reset();
        if (clock.milliseconds() == 150){

            drive.ringHopper.setPosition(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 250){
            drive.ringHopper.setPosition(0.9);
            clock.reset();
        }

        if (clock.milliseconds() == 150){

            drive.ringHopper.setPosition(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 250){
            drive.ringHopper.setPosition(0.9);
            clock.reset();
        }

        if (clock.milliseconds() == 150){

            drive.ringHopper.setPosition(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 250){
            drive.ringHopper.setPosition(0.9);
            clock.reset();
        }

        if (clock.milliseconds() == 150){

            drive.ringHopper.setPosition(0.5);
            clock.reset();
        }

        if (clock.milliseconds() == 250){
            drive.FlyWheel1.setVelocity(0);
            drive.FlyWheel2.setVelocity(0);

        }

    }
}

