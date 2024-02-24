package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepy {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90.00)))
                        .lineToConstantHeading(new Vector2d(16, -41))
                        .lineToLinearHeading(new Pose2d(10, -38, Math.toRadians(-35)))
                        .lineToLinearHeading(new Pose2d(50, -38, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(51, -58))
                        .lineToLinearHeading(new Pose2d(51.01, -58.01, Math.toRadians(90)))
                        .build());




                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                .start();
    }
}