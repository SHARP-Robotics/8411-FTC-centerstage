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
public class RedBACKDROPAuto extends OpMode {
    private OCVVisionProc drawProcessor;
    private VisionPortal visionPortal;
    private Servo pixelDrop = null;
    private Servo backPixelDrop = null;
    int positionDetect = 0;

    @Override
    public void init() {
        drawProcessor = new OCVVisionProc();
        pixelDrop = hardwareMap.get(Servo.class, "puDrop");
        backPixelDrop = hardwareMap.get(Servo.class, "p3Drop");
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

                Pose2d startPoseL = new Pose2d(11, -61, Math.toRadians(-90));

                drive.setPoseEstimate(startPoseL);

                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToConstantHeading(new Vector2d(13, -41.5))
                        .lineToLinearHeading(new Pose2d(13.5, -40.5, Math.toRadians(-35)))

                        .addDisplacementMarker(17.6,() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToLinearHeading(new Pose2d(20, -35, Math.toRadians(-90)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToLinearHeading(new Pose2d(40, -23, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.8);
                        })

                        .lineToConstantHeading(new Vector2d(51, -23))
                        .lineToConstantHeading(new Vector2d(49.5, -23))
                        .lineToConstantHeading(new Vector2d(45, -23))
                        .lineToConstantHeading(new Vector2d(45, -60))

                        .build();

                drive.followTrajectorySequence(trajL);

                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 3:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseR = new Pose2d(11, -61, Math.toRadians(-90));

                drive.setPoseEstimate(startPoseR);
                TrajectorySequence trajR = drive.trajectorySequenceBuilder(startPoseR)
                        .lineToConstantHeading(new Vector2d(27.00, -58.00))
                        .lineToConstantHeading(new Vector2d(27, -35))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(27.00, -42.00))
                        .lineToConstantHeading(new Vector2d(28.00, -45.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToLinearHeading(new Pose2d(41, -34, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.8);
                        })
                        .lineToConstantHeading(new Vector2d(50, -34.5))
                        .lineToConstantHeading(new Vector2d(46, -34.5))
                        .lineToConstantHeading(new Vector2d(46, -60))
                        .build();
                drive.followTrajectorySequence(trajR);
                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 2:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseM = new Pose2d(11, -61, Math.toRadians(-90));

                // enderman
                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)
                        .lineToConstantHeading(new Vector2d(15.00, -33.00))
                        .lineToConstantHeading(new Vector2d(15.00, -38))

                        .addDisplacementMarker(2,() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToLinearHeading(new Pose2d(40, -28, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.78);
                        })

                        .lineToConstantHeading(new Vector2d(51.00, -28))
                        .lineToConstantHeading(new Vector2d(49, -28))
                        .lineToConstantHeading(new Vector2d(45, -28))
                        .lineToConstantHeading(new Vector2d(45, -60))
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
