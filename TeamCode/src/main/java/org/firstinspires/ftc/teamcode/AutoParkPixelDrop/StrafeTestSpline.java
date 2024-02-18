package org.firstinspires.ftc.teamcode.AutoParkPixelDrop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


@Config
@Autonomous(group = "drive")
@Disabled
public class StrafeTestSpline extends LinearOpMode {
    private CRServo pixelDrop = null;

    @Override
    public void runOpMode() {
        waitForStart();
        pixelDrop = hardwareMap.get(CRServo.class, "pDrop");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(11, 61, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(11, 10.5), Math.toRadians(180))
                .build();
        drive.setPoseEstimate(startPose);


        if(isStopRequested()) return;
        if(isStarted()) {
            drive.followTrajectorySequence(traj1);
        }
    }
}

