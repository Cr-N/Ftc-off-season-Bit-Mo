package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        double RIGHT_STRAFE_1_X=-40;
        double RIGHT_STRAFE_1_Y=45;
        double RIGHT_SPILNE_1_X = -37;
        double RIGHT_SPLINE_1_Y = 36;
        double RIGHT_SPLINE_1_HEADING = Math.toRadians(270+60);
        double RIGHT_SPLINE_1_TANGENT = 0;
        double RIGHT_WAIT_SECONDS_1 = 0.5;
        double RIGHT_STRAFE_1_VECTOR_X = -46.5;
        double RIGHT_STRAFE_1_VECTOR_Y = -43;
        double RIGHT_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
        double RIGHT_SPLINE_2_X = -35.4;
        double RIGHT_SPLINE_2_Y = 62;
        double RIGHT_SPLINE_2_HEADING = Math.PI;
        double RIGHT_SPLINE_2_TANGENT = 0;
        double RIGHT_STRAFE_2_VECTOR_X = 5;
        double RIGHT_STRAFE_2_VECTOR_Y = 59.9;
        double RIGHT_SPLINE_3_X = 48.4;
        double RIGHT_SPLINE_3_Y = 39; // -22.4
        double RIGHT_SPLINE_3_TANGENT = 0;
        double RIGHT_WAIT_SECONDS_2 = 2;
        double RIGHT_PARK_X = 50;
        double RIGHT_PARK_Y = 60;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 25.85)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.2, 63.2, 3*Math.PI/2))
                //Spike mark LEFT
                .strafeToConstantHeading(new Vector2d(RIGHT_STRAFE_1_X,RIGHT_STRAFE_1_Y))
                .splineToLinearHeading(new Pose2d(RIGHT_SPILNE_1_X,RIGHT_SPLINE_1_Y,RIGHT_SPLINE_1_HEADING),RIGHT_SPLINE_1_TANGENT)
                // .splineTo(new Vector2d(SPILNE_1_X,SPLINE_1_Y),SPLINE_1_TANGENT)
                .waitSeconds(RIGHT_WAIT_SECONDS_2)

                //Go back a little
                //.strafeToSplineHeading(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y),STRAFE_1_HEADING)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go to leaving spot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_2_X,RIGHT_SPLINE_2_Y,RIGHT_SPLINE_2_HEADING),RIGHT_SPLINE_2_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go forward a little(we cross from long to short)
                .strafeToConstantHeading(new Vector2d(RIGHT_STRAFE_2_VECTOR_X,RIGHT_STRAFE_2_VECTOR_Y))
                //.waitSeconds(WAIT_SECONDS_2)

                // Spline to Backboard on the RIGHT side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(RIGHT_SPLINE_3_X,RIGHT_SPLINE_3_Y),RIGHT_SPLINE_3_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Deploy Yellow
                .waitSeconds(RIGHT_WAIT_SECONDS_2)

                //Park in the corner
                //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                .splineToConstantHeading(new Vector2d(RIGHT_PARK_X,RIGHT_PARK_Y),RIGHT_SPLINE_3_TANGENT)


                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}