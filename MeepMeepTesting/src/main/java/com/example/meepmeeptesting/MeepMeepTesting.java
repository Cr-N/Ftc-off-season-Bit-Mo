package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        double LINE_TO_Y_1_Y = -38;
        double WAIT_SECONDS_1 = 1;
        double LINE_TO_Y_2_Y = -37.7;
        double SPLINE_1_X = 58;
        double SPLINE_1_Y = -30;
        double SPLINE_1_HEADING = Math.PI;
        double SPLINE_1_TANGENT = 0;
        double PARK_X = 50;
        double PARK_Y = -60;
        double PARK_TANGENT = 0;
        double WAIT_SECONDS_2 = 2;
        double WAIT_SECONDS_3 = 2;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 25.85)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -63.2, Math.PI/2))

                //.waitSeconds(WAIT_SECONDS_1)
                // SPIKE MARK **MIDDLE**
                .lineToY(LINE_TO_Y_1_Y)
                .waitSeconds(WAIT_SECONDS_1)

                .lineToYConstantHeading(LINE_TO_Y_2_Y)

                // Spline catre Backdrop 1
                        .setReversed(true)
                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)
                //.afterTime(0.2,master.Score_Yellow())
                .waitSeconds(WAIT_SECONDS_2)

                .waitSeconds(WAIT_SECONDS_3)
                .splineToConstantHeading(new Vector2d(PARK_X,PARK_Y),PARK_TANGENT)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}