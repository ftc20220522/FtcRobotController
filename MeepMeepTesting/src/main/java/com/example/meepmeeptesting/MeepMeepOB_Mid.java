package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepOB_Mid {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16, -61, Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(22,-24))
                                .lineToConstantHeading(new Vector2d(40,-24))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(34,-15))
                                .lineToConstantHeading(new Vector2d(44,-10))
                                .lineToConstantHeading(new Vector2d(-44,-10))
                                .lineToConstantHeading(new Vector2d(-46,-31))
                                .lineToConstantHeading(new Vector2d(-53,-31))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}