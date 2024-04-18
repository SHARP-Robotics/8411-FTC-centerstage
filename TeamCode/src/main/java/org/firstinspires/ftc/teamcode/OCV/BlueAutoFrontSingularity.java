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

/*
IMPORTANT
Left has skew for this, its not super consistent
 */

@Autonomous
public class BlueAutoFrontSingularity extends OpMode {
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
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
                Pose2d startPoseL = new Pose2d(-36, 61, Math.toRadians(90));
                drive.setPoseEstimate(startPoseL);
                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToConstantHeading(new Vector2d(-36, 51))
                        .lineToLinearHeading(new Pose2d(-34.5, 32.00, Math.toRadians(160)))
                        .lineToConstantHeading(new Vector2d(-42, 36.00))

                        .addDisplacementMarker(16, () -> {
                            pixelDrop.setPosition(0.1);
                        })


                        .addDisplacementMarker(16, () -> {
                            backPixelDrop.setPosition(0.6);
                        })

                        .lineToLinearHeading(new Pose2d(-42, 57.00, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(44, 57))
                        .lineToConstantHeading(new Vector2d(48.50, 44.5))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.78);
                        })

                        .lineToConstantHeading(new Vector2d(48, 44.5))
                        .lineToConstantHeading(new Vector2d(47, 44.5))



                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(44, 60))
                        .lineToConstantHeading(new Vector2d(59, 60))



                        .build();

                drive.followTrajectorySequence(trajL);

                visionPortal.setProcessorEnabled(drawProcessor, false);
                positionDetect = 0;
                break;
            case 3:
                visionPortal.setProcessorEnabled(drawProcessor, false);
                visionPortal.stopStreaming();
                drive = new SampleMecanumDrive(hardwareMap);

                Pose2d startPoseR = new Pose2d(-36, 61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseR);
                TrajectorySequence trajR = drive.trajectorySequenceBuilder(startPoseR)
                        .waitSeconds(1.5)

                        .lineToConstantHeading(new Vector2d(-42.5, 45))

                        .addDisplacementMarker(2, () -> {
                            pixelDrop.setPosition(0.6);
                        })


                        .addDisplacementMarker(2,() -> {
                            backPixelDrop.setPosition(0.6);
                        })

                        .lineToLinearHeading(new Pose2d(-42, 57.00, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(45, 57))
                        .lineToConstantHeading(new Vector2d(49.00, 29))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.78);
                        })

                        .lineToConstantHeading(new Vector2d(48, 29))
                        .lineToConstantHeading(new Vector2d(45, 29))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(45, 60))
                        .lineToConstantHeading(new Vector2d(60, 60))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .addDisplacementMarker(17, () -> {
                            pixelDrop.setPosition(0);
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

                telemetry.addData("Testing back servo pos", backPixelDrop.getPosition());

                Pose2d startPoseM = new Pose2d(-34.5, 61, Math.toRadians(90));
                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)
                        .lineToConstantHeading(new Vector2d(-37, 36.00))

                        .addDisplacementMarker(2, () -> {
                            pixelDrop.setPosition(0);
                        })


                        .addDisplacementMarker(2,() -> {
                            backPixelDrop.setPosition(0.5);
                        })

                        .lineToLinearHeading(new Pose2d(-42, 57.00, Math.toRadians(180)))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(45, 57))
                        .lineToConstantHeading(new Vector2d(50.00, 35.5))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0.78);
                        })

                        .lineToConstantHeading(new Vector2d(49, 35.5))
                        .lineToConstantHeading(new Vector2d(45, 35.5))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(1);
                        })

                        .lineToConstantHeading(new Vector2d(45, 60))
                        .lineToConstantHeading(new Vector2d(60, 60))

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
