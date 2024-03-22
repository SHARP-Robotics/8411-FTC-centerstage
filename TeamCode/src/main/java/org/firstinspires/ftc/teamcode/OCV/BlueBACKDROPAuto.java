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
public class BlueBACKDROPAuto extends OpMode {
    private OCVVisionProc drawProcessor;
    private VisionPortal visionPortal;
    private Servo pixelDrop = null;
    private Servo backPixelDrop = null;
    int positionDetect = 0;


    @Override
    public void init() {
        drawProcessor = new OCVVisionProc();

        pixelDrop = hardwareMap.get(Servo.class, "pDrop");
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
                        .lineToConstantHeading(new Vector2d(26.00, 58.00))
                        .lineToConstantHeading(new Vector2d(25.00, 31.00))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(25.00, 41.5))
                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })
                        .lineToLinearHeading(new Pose2d(40.00, 41.5, Math.toRadians(-180)))
                        .lineToConstantHeading(new Vector2d(49.75, 41.5))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.79);
                        })

                        .lineToConstantHeading(new Vector2d(46, 41.5))
                        .lineToConstantHeading(new Vector2d(46, 60))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
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

                        .lineToLinearHeading(new Pose2d(24.00, 38.00, Math.toRadians(-90)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToLinearHeading(new Pose2d(40.00, 33.00, Math.toRadians(-180)))
                        .lineToConstantHeading(new Vector2d(49.75, 33))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.79);
                        })

                        .lineToConstantHeading(new Vector2d(50, 33))
                        .lineToConstantHeading(new Vector2d(45, 33))
                        .lineToConstantHeading(new Vector2d(45, 60))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
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


                        .addDisplacementMarker(6.25, () -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(11.00, 38.5))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })
                        .lineToLinearHeading(new Pose2d(40.00, 34.5, Math.toRadians(-180)))
                        .lineToConstantHeading(new Vector2d(49.75, 34.5))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.79);
                        })

                        .lineToConstantHeading(new Vector2d(46, 34.5))
                        .lineToConstantHeading(new Vector2d(46, 60))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
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