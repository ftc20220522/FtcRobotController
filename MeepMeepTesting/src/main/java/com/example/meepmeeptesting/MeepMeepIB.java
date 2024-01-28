package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepIB {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
//        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be blue
//                .setColorScheme(new ColorSchemeRedLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(270)))
//                                .lineToConstantHeading(new Vector2d(14.5,31))
//                                .lineToConstantHeading(new Vector2d(10,31))
//                                .lineToConstantHeading(new Vector2d(10,10))
//                                .turn(Math.toRadians(90))
//                                .lineToConstantHeading(new Vector2d(44,10))
//                                .lineToConstantHeading(new Vector2d(46,44))
//                                .lineToConstantHeading(new Vector2d(46,58))
//                                .lineToConstantHeading(new Vector2d(60,58))
//                                .build()
//                );

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(14,40))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(14,33))
                                .lineToConstantHeading(new Vector2d(46,36))
                                .lineToConstantHeading(new Vector2d(46,58))
                                .lineToConstantHeading(new Vector2d(60,58))
                                .build()
                );

        // Declare out third bot
        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(14,34))
                                .turn(Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(9,34))
                                .lineToConstantHeading(new Vector2d(16,34))
                                .turn(Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(46,28))
                                .lineToConstantHeading(new Vector2d(46,58))
                                .lineToConstantHeading(new Vector2d(60,58))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
//                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .addEntity(myThirdBot)
                .start();
    }
}