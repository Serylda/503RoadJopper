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

public class DoALittleJopping extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        DROP,   // Then, follow a lineTo() trajectory
        TRAJ_2,         // Then we want to do a point turn
        GRAB,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    @Override
    public void runOpMode() {
    }
}

