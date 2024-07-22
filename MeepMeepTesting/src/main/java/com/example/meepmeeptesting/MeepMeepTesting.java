package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
         double LINE_TO_Y_1_Y = -35;
         double WAIT_SECONDS_1 = 1;
         double LINE_TO_Y_2_Y = -37.7;
         double LINE_TO_Y_3_Y = -42.2;
         double LINE_TO_Y_3_HEADING = 0;
         double SPLINE_1_X = 48.4;
         double SPLINE_1_Y = -35.4;
         double SPLINE_1_HEADING = Math.PI;
         double SPLINE_1_TANGENT = 0;
         double STRAFE_1_VECTOR_X = 46;
         double STRAFE_1_VECTOR_Y = -60;
         double WAIT_SECONDS_2 = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 25.85)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -61.5, Math.PI/2))
                // SPIKE MARK **MIDDLE**
                .lineToY(LINE_TO_Y_1_Y)

                .waitSeconds(WAIT_SECONDS_1)

                // AJUSTARE sa nu dai in PIXELUL MOV

                .lineToYConstantHeading(LINE_TO_Y_2_Y)

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)

                //.splineToConstantHeading(new Vector2d(SPLINE_1_X, SPLINE_1_Y), SPLINE_1_TANGENT) // Spline catre Backdrop 1 tangent 0
                // deploy pixel 1
                .waitSeconds(WAIT_SECONDS_2)

                .strafeTo(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}