package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 15.5)
                .setDimensions(11.95, 14.80)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50.5, -50.5, Math.toRadians(234.046)))
                .waitSeconds(2)
                .strafeToConstantHeading(
                    new Vector2d(-12, -10)
                )
                .strafeToLinearHeading(
                        new Vector2d(-11, -50),
                        Math.toRadians(270),
                        new TranslationalVelConstraint(30),
                        new ProfileAccelConstraint(-80, 80)
                )

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}