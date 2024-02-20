package org.firstinspires.ftc.teamcode.OCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous
public class BlueAutoFront extends OpMode {
    private OCVVisionProc drawProcessor;
    private VisionPortal visionPortal;
    private Servo pixelDrop = null;

    @Override
    public void init() {
        drawProcessor = new OCVVisionProc();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), drawProcessor);
        visionPortal.resumeStreaming();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

        visionPortal.resumeStreaming();
        visionPortal.setProcessorEnabled(drawProcessor, true);
    }

    @Override
    public void loop()  {
        pixelDrop = hardwareMap.get(Servo.class, "pDrop");

        telemetry.addData("Identified", drawProcessor.getSelection());


        switch (drawProcessor.getSelection()) {
            case LEFT:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseL = new Pose2d(-36, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseL);

                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToLinearHeading(new Pose2d(-24.00, 31.00, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(10, 31))
                        .splineTo(new Vector2d(48, 61), Math.toRadians(0))
                        .build();

                drive.followTrajectorySequence(trajL);

                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.close();
                stop();
                requestOpModeStop();
                break;
            case RIGHT:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseR = new Pose2d(-36, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseR);
                TrajectorySequence trajR = drive.trajectorySequenceBuilder(startPoseR)
                        .lineToConstantHeading(new Vector2d(-46.00, 30.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(11, 10))
                        .splineTo(new Vector2d(48, 61), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequence(trajR);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.close();
                stop();
                requestOpModeStop();
                break;
            case MIDDLE:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseM = new Pose2d(-36, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)
                        .lineToConstantHeading(new Vector2d(-36.00, 26.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(11, 10))
                        .splineTo(new Vector2d(48, 61), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequence(trajM);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.close();
                stop();
                requestOpModeStop();
                break;
            case NONE:
                visionPortal.setProcessorEnabled(drawProcessor, true);
                visionPortal.resumeStreaming();
                break;
        }
    }
}
