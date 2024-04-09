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
public class RedAutoFront extends OpMode {
    private OCVVisionProc drawProcessor;
    private VisionPortal visionPortal;
    private Servo pixelDrop = null;
    private Servo backPixelDrop = null;
    int positionDetect = 0;

    @Override
    public void init() {
        pixelDrop = hardwareMap.get(Servo.class, "pDrop");
        backPixelDrop = hardwareMap.get(Servo.class, "p3Drop");
        drawProcessor = new OCVVisionProc();
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

        visionPortal.resumeStreaming();
        visionPortal.setProcessorEnabled(drawProcessor, true);
    }

    @Override
    public void loop()  {
        telemetry.addData("Identified", drawProcessor.getSelection());


        switch (positionDetect) {
            case 1:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseL = new Pose2d(-36,-61, Math.toRadians(-90));

                drive.setPoseEstimate(startPoseL);

                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToConstantHeading(new Vector2d(-52, -44))

                        .addDisplacementMarker(2, () -> {
                            pixelDrop.setPosition(0);
                        })


                        .addDisplacementMarker(2,() -> {
                            backPixelDrop.setPosition(0.6);
                        })

                        .lineToConstantHeading(new Vector2d(-52, -42))
                        .lineToLinearHeading(new Pose2d(-34.5, -59.00, Math.toRadians(-360)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(45, -59))
                        .lineToConstantHeading(new Vector2d(51.00, -28))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.78);
                        })

                        .lineToConstantHeading(new Vector2d(49, -28))
                        .lineToConstantHeading(new Vector2d(45, -28))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(45, -60))
                        .lineToConstantHeading(new Vector2d(60, -60))

                        .build();

                drive.followTrajectorySequence(trajL);

                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 3:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseR = new Pose2d(-34.5, -61, Math.toRadians(-90));

                drive.setPoseEstimate(startPoseR);
                TrajectorySequence trajR = drive.trajectorySequenceBuilder(startPoseR)
                        .lineToConstantHeading(new Vector2d(-34.5, -51))
                        .lineToConstantHeading(new Vector2d(-35.1, -38))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToLinearHeading(new Pose2d(-35.0, -38.00, Math.toRadians(220)))
                        .lineToConstantHeading(new Vector2d(-40, -35))
                        .lineToConstantHeading(new Vector2d(-38, -55.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .build();
                drive.followTrajectorySequence(trajR);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 2:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseM = new Pose2d(-34.5, -61, Math.toRadians(-90));

                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)


                        .build();
                drive.followTrajectorySequence(trajM);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 0:
                break;
        }
    }
}

