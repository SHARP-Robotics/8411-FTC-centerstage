package org.firstinspires.ftc.teamcode.OCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
public class BlueAutoBack extends OpMode {
    private OCVVisionProc drawProcessor;
    private VisionPortal visionPortal;
    private Servo pixelDrop = null;
    private Servo backPixelDrop = null;
    int positionDetect = 0;


    @Override
    public void init() {
        drawProcessor = new OCVVisionProc();

        pixelDrop = hardwareMap.get(Servo.class, "puDrop");
        backPixelDrop = hardwareMap.get(Servo.class, "pDrop");
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), drawProcessor);
        visionPortal.resumeStreaming();
    }

    @Override
    public void init_loop() {
        visionPortal.resumeStreaming();
        visionPortal.setProcessorEnabled(drawProcessor, true);

        switch (drawProcessor.getSelection()) {
            case LEFT:
                positionDetect = 1;
                break;
            case MIDDLE:
                positionDetect = 2;
                break;
            case RIGHT:
                positionDetect = 3;
                break;
            case NONE:
                positionDetect = 0;
                break;
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Identified", positionDetect);

        switch (positionDetect) {
            case 1:
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseL = new Pose2d(11, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseL);

                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToConstantHeading(new Vector2d(19.00, 61.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(19.00, 44.00))
                        .lineToConstantHeading(new Vector2d(44, 56))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
                        })

                        .build();

                drive.followTrajectorySequence(trajL);

                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 3:
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseR = new Pose2d(11, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseR);
                TrajectorySequence trajR = drive.trajectorySequenceBuilder(startPoseR)
                        .lineToConstantHeading(new Vector2d(11, 54))
                        .lineToLinearHeading(new Pose2d(8.50, 42.00, Math.toRadians(60)))

                        .addDisplacementMarker(7, () -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToLinearHeading(new Pose2d(48.00, 57.00, Math.toRadians(100)))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
                        })

                        .build();
                drive.followTrajectorySequence(trajR);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 2:
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseM = new Pose2d(11, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)
                        .lineToConstantHeading(new Vector2d(11.00, 34.00))

                        .addDisplacementMarker(12, () -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(48.00, 57.00))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
                        })

                        .build();
                drive.followTrajectorySequence(trajM);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 0:
                visionPortal.setProcessorEnabled(drawProcessor, true);
                break;

        }
    }
}