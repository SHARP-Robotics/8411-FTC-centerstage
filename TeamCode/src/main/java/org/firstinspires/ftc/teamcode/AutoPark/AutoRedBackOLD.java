package org.firstinspires.ftc.teamcode.AutoPark;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
@Disabled
public class AutoRedBackOLD extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, -37, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .back(36)
                .build();

        /*
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();
        */

        if(isStopRequested()) return;
        if(isStarted()) {

            drive.followTrajectory(traj1);
            // drive.followTrajectory(traj2);
        }
    }
}

