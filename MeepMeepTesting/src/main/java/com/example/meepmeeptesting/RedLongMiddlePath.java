package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLongMiddlePath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.2, -63.2, Math.PI/2))


                //Spike mark MIDDLE
                .strafeToConstantHeading(new Vector2d(-36,-33.5))

                .waitSeconds(0.3)
                //Go back a little
                .strafeToSplineHeading(new Vector2d(-46.5,-38),Math.PI/2+Math.PI/6)
                //Go to stack
                .splineToLinearHeading(new Pose2d(-56,-35.7,Math.PI),2) //-36.5

                // Intake one pixel
                .waitSeconds(1)

                // Spline to Backboard on the MIDDLE side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(20,-35.4),0)

                                .splineToLinearHeading(new Pose2d(48.4,-35.4,0),0)

                //Go to stack
               // .splineToLinearHeading(new Pose2d(-56,-35.7,Math.PI),2)
                                .strafeToConstantHeading(new Vector2d(-32.5,-35.7)) //-56

                        //.strafeToLinearHeading(new Vector2d(-56,-35.7),Math.PI
                        .setReversed(true)
                        
                        .strafeToSplineHeading(new Vector2d(-56,-35.7),Math.PI)
                //Park in the corner
                //.strafeTo(new Vector2d(48.3,-58))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)

                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}