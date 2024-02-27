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
    private Servo backPixelDrop = null;
    private Servo p3DropSOFHSIEHfi = null;
    int positionDetect = 0;

    @Override
    public void init() {
        pixelDrop = hardwareMap.get(Servo.class, "pDrop");
        backPixelDrop = hardwareMap.get(Servo.class, "puDrop");
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

                Pose2d startPoseL = new Pose2d(-36, 61, Math.toRadians(90));
                TrajectorySequence trajL = drive.trajectorySequenceBuilder(startPoseL)
                        .lineToLinearHeading(new Pose2d(-34.5, 42.00, Math.toRadians(160)))
                        .lineToConstantHeading(new Vector2d(-34.5, 40))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(48, 58))


                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
                        })

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
                        .lineToConstantHeading(new Vector2d(-42.00, 61.00))
                        .lineToConstantHeading(new Vector2d(-42, 31))

                        .addDisplacementMarker(() -> {
                            pixelDrop.setPosition(0);
                        })

                        .lineToConstantHeading(new Vector2d(-42, 43))
                        .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(48, 58))

                        .addDisplacementMarker(() -> {
                            backPixelDrop.setPosition(0);
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

                Pose2d startPoseM = new Pose2d(-34.5, 61, Math.toRadians(90));
                drive.setPoseEstimate(startPoseM);
                TrajectorySequence trajM = drive.trajectorySequenceBuilder(startPoseM)

                        .addDisplacementMarker(14, () -> {
                            pixelDrop.setPosition(0);
                        })

                    .lineToConstantHeading(new Vector2d(-34.5, 29))
                    .lineToConstantHeading(new Vector2d(-34.5, 34))
                    .lineToConstantHeading(new Vector2d(-34.5, 58))
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
