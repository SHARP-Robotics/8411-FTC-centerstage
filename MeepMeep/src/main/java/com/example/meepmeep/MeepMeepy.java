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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34.5, -61, Math.toRadians(-90.00)))
                        .lineToConstantHeading(new Vector2d(-52, -45))
                        .lineToLinearHeading(new Pose2d(-42, -57.00, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(45, -57))
                        .lineToConstantHeading(new Vector2d(51.00, -31.5))
                        .lineToConstantHeading(new Vector2d(49, -31.5))
                        .lineToConstantHeading(new Vector2d(45, -31.5))
                        .lineToConstantHeading(new Vector2d(45, -60))
                        .lineToConstantHeading(new Vector2d(60, -60))


                        .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                .start();
    }
}