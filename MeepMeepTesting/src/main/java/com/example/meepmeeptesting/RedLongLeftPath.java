package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


/// TODO: Ask the team how to improve this to make it 2+5 (Red-Long-Left)
// This is currently a 2 + 3 (I tried to make another cycle but it surpasses 30s - to be seen when tuning our robot
public class RedLongLeftPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 25.58)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.2, -63.2, Math.PI/2))


                        //Spike mark LEFT
                        .splineToLinearHeading(new Pose2d(-46.5,-35,Math.PI/2),2)

                        .waitSeconds(0.5)
                        //Go back a little
                        .strafeToSplineHeading(new Vector2d(-46.5,-38),Math.PI/2+Math.PI/6)
                        //Go to stack
                        .splineToLinearHeading(new Pose2d(-56,-36.5,Math.PI),2)

                        // Intake one pixel
                        .waitSeconds(1)

                        // Go to leaving spot
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-35.4,-58.9,0),0)

                        // Go forward a little(we cross from long to short)
                        .strafeToConstantHeading(new Vector2d(0,-58.9))

                        // Spline to Backboard on the LEFT side
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(48.4,-30.4),0)

                        // Deploy Yellow + 1st White
                        .waitSeconds(1)

                        //Align to get back to the stack(on the bottom)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(13,-58.9,Math.PI),4.3)

                        //Spline to Stack for intake
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(-56,-36.5),1.75)

                        // Intake 2nd and 3rd Pixels
                        .waitSeconds(1)

                        //Go to leaving spot
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-35.4,-58.9,0),0)

                        // Go forward
                        .strafeToConstantHeading(new Vector2d(0,-58.9))

                        //Spline to Backboard on the RIGHT side(closer to us so It's faster)
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(48.4,-40.4),0)

                        //Deploy 2nd and 3rd Pixels
                        .waitSeconds(1)

                        //Park in the corner
                        .strafeTo(new Vector2d(48.3,-58))
                        .splineToConstantHeading(new Vector2d(48.3,-58),0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)

                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}