package org.firstinspires.ftc.teamcode.AutoPark;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@Autonomous(group = "drive")
@Disabled
public class AutoBlueBackOLD extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, 61, Math.toRadians(180));

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
        }

    }
}

