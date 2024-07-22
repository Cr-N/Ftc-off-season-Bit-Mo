package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedShortMiddlePath2plus4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 25.85)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -63.2, Math.PI/2))

                // SPIKE MARK **MIDDLE**
                .lineToY(-33.2)

                // AJUSTARE sa nu dai in PIXELUL MOV
                .lineToYConstantHeading(-37.7) // -40.2
                 .lineToYSplineHeading(-42.2,0) //-42.2

                // Spline catre Backdrop 1
                .splineToConstantHeading(new Vector2d(48.4, -35.4), 0) // Spline catre Backdrop 1 tangent 0

                // deploy pixel 1
                .waitSeconds(1)

                //  mergi in spate putin
                .lineToXSplineHeading(42.4,Math.PI/2)

                // Mergi la Pozitia de unde PLECI CATRE STACK
                .strafeToLinearHeading(new Vector2d(44.5 , -12),Math.PI)

                // Mergi la STACK x = -56 y = -12 !!!!! VERY IMPORTANT
                .strafeToConstantHeading(new Vector2d(-56 , -12))

                // intake pixel 1
                .waitSeconds(1)

                // mergi inapoi la pozitia de unde ai plecat ca sa pleci catre STACK
                .lineToX(-50) // -43
                .lineToXLinearHeading(-43,0)
                .lineToXConstantHeading(44.5)

                // Spline catre Backdrop 2
                .splineToConstantHeading(new Vector2d(48.4, -35.4), 0)

                // deploy pixel 2
                .waitSeconds(1)

                //  mergi in spate putin
                .lineToXSplineHeading(42.4,Math.PI/2)

                // Mergi la Pozitia de unde PLECI CATRE STACK
                .strafeToLinearHeading(new Vector2d(44.5 , -12),Math.PI)

                // Mergi la STACK x = -56 y = -12 !!!!! VERY IMPORTANT
                .strafeToConstantHeading(new Vector2d(-56 , -12))

                // intake pixel 2
                .waitSeconds(1)

                // mergi inapoi la pozitia de unde ai plecat ca sa pleci catre STACK
                .lineToX(-50)
                .lineToXLinearHeading(-43,0)
                .lineToXConstantHeading(44.5)


                // Spline catre Backdrop 2
                .splineToConstantHeading(new Vector2d(48.4, -35.4), 0)

                // deploy pixel 3
                .waitSeconds(1)

                // Mergi la STACK x = -56 y = -12 !!!!! VERY IMPORTANT
                .strafeToConstantHeading(new Vector2d(47.3 , -12))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}