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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.2, -63.2, Math.PI/2))

                //Spike mark LEFT
                .splineToLinearHeading(new Pose2d(-46.5,-35,Math.PI/2),2)

                .waitSeconds(0.5)
                //Go back a little
                .strafeToSplineHeading(new Vector2d(-46.5,-38),Math.PI/2+Math.PI/6)
                //Go to stack
                // Intake one pixel

                // Go to leaving spot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35.4,-58.9, Math.PI),0)

                // Go forward a little(we cross from long to short)
                .strafeToConstantHeading(new Vector2d(0,-58.9))

                // Spline to Backboard on the LEFT side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48.4,-30.4),0)

                // Deploy Yellow
                .waitSeconds(1)

                //Park in the corner
                .strafeTo(new Vector2d(50,-60))

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}