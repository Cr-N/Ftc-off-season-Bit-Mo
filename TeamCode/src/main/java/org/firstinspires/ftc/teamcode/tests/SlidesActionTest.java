package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubsystemsWithActions.SlidesWithActionsForAutos;

@TeleOp
public class SlidesActionTest extends LinearOpMode {
    SlidesWithActionsForAutos slides;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = new SlidesWithActionsForAutos(hardwareMap);
        waitForStart();
        if(isStopRequested())return;
                Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                                slides.FORAUTO_Move_To_LEVEL_1(),
                                slides.FORAUTO_Move_To_LEVEL_2(),
                                slides.FORAUTO_Move_To_LEVEL_3(),
                                slides.FORAUTO_Move_To_LEVEL_4(),
                                slides.FORAUTO_Move_To_LEVEL_3(),
                                slides.FORAUTO_Move_To_LEVEL_2(),
                                slides.FORAUTO_Move_To_LEVEL_1(),
                                slides.FORAUTO_Move_To_LEVEL_INTAKE_POSITION()
                        )
                )
        );
    }
}
