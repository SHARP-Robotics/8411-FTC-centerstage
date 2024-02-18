package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@Autonomous(group = "drive")
@Disabled
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, 61, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(14.27, 31.88), Math.toRadians(-87.05))
                .splineTo(new Vector2d(30.98, 4.24), Math.toRadians(-58.83))
                .splineTo(new Vector2d(17.83, -23.41), Math.toRadians(244.55))
                .splineTo(new Vector2d(27.64, -48.37), Math.toRadians(-68.55))
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

