package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedShortMiddle2plus6 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11.5, -63.2, Math.PI/2))
//Caz Mijloc Short Red daca suntem singuri pe teren :)
                .lineToY(-34)
                .waitSeconds(0.5) //Pixel mov pus
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(45, -35.2, 0), Math.toRadians(20))
                .waitSeconds(0.5) //Ajunge la tabla cu pixel galben
//Ciclu 1
                .strafeToLinearHeading(new Vector2d(40, -35), Math.PI)
                .splineToConstantHeading(new Vector2d(-55, -30), Math.toRadians(170))
                .waitSeconds(0.5) // Intake 2 Pixeli albi din stack-uri
                .strafeToLinearHeading(new Vector2d(-50, -35), 0)
                .strafeToConstantHeading(new Vector2d(45, -35.2))
                .waitSeconds(0.5)//2 Pixeli albi pusi pe tabla (2+1)
//Ciclu 2
                .strafeToLinearHeading(new Vector2d(40, -35), Math.PI)
                .splineToConstantHeading(new Vector2d(-55, -30), Math.toRadians(170))
                .waitSeconds(0.5) // Intake 2 Pixeli albi din stack-uri
                .strafeToLinearHeading(new Vector2d(-50, -35), 0)
                .strafeToConstantHeading(new Vector2d(45, -35.2))
                .waitSeconds(0.5)//2 Pixeli albi pusi pe tabla (4+1)
//Ciclu 3
                .strafeToLinearHeading(new Vector2d(40, -35), Math.PI)
                .splineToConstantHeading(new Vector2d(-55, -30), Math.toRadians(170))
                .waitSeconds(0.5) //Intake 2 Pixeli albi de la stack-uri
                .strafeToLinearHeading(new Vector2d(-50, -35), 0)
                .strafeToConstantHeading(new Vector2d(45, -35.2)) //2 Pixeli albi pusi pe tabla (6+1)
//Sfarsit + Parcare
                        .strafeTo(new Vector2d(45, -50))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
