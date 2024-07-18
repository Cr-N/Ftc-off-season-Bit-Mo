package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -63.2, Math.PI/2))
                // SPIKE MARK **LEFT**
                .lineToY(-40.2)

                .splineToLinearHeading(new Pose2d(8.50,-36.2,2.6179938779914944),1)

                .waitSeconds(2)

                // Mergi putin in spate ca sa nu dai in pixelul MOV
                .strafeTo(new Vector2d(13,-36))

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(48.4,-30.4,0),0)

                .waitSeconds(3)

                // Parcare
                .strafeTo(new Vector2d(43,-58))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}