package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
// Still work to do
public class BlueMiddleShortPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())  // Set the robot color to blue
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, 63.2, -Math.PI/2))  // Invert y-coordinate of the starting pose

                .lineToY(33.2)
                .lineToYConstantHeading(37.7)  // Invert y-coordinate
                .lineToYSplineHeading(42.2, 0)  // Invert y-coordinate

                .splineToConstantHeading(new Vector2d(48.4, 35.4), 0)  // Invert y-coordinate
                .waitSeconds(1)
                .lineToXSplineHeading(42.4, -Math.PI/2)
                .strafeToLinearHeading(new Vector2d(44.5, 12), Math.PI)  // Invert y-coordinate
                .strafeToConstantHeading(new Vector2d(-56, 12))  // Invert y-coordinate
                .waitSeconds(1)
                .lineToX(-50)
                .lineToXLinearHeading(-43, 0)
                .lineToXConstantHeading(44.5)

                .splineToConstantHeading(new Vector2d(48.4, 35.4), 0)  // Invert y-coordinate
                .waitSeconds(1)
                .lineToXSplineHeading(42.4, -Math.PI/2)
                .strafeToLinearHeading(new Vector2d(44.5, 12), Math.PI)  // Invert y-coordinate
                .strafeToConstantHeading(new Vector2d(-56, 12))  // Invert y-coordinate
                .waitSeconds(1)
                .lineToX(-50)
                .lineToXLinearHeading(-43, 0)
                .lineToXConstantHeading(44.5)

                .splineToConstantHeading(new Vector2d(48.4, 35.4), 0)  // Invert y-coordinate
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(47.3, 12))  // Invert y-coordinate
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
