package org.firstinspires.ftc.teamcode.AutoParkPixelDrop;

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
public class AutoBlueFrontPD extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 61, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(37)
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(13)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(82)
                .build();


        if(isStopRequested()) return;
        if(isStarted()) {

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }
    }
}

