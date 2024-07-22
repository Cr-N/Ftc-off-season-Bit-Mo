package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.SlidesWithActionsForAutos;

@Config
@Autonomous
public class REDSHORTMIDDLE extends LinearOpMode {

    public static double LINE_TO_Y_1_Y = -35;
    public static double WAIT_SECONDS_1 = 1;
    public static double LINE_TO_Y_2_Y = -37.7;

    public static double SPLINE_1_X = 52;
    public static double SPLINE_1_Y = -30;
    public static double SPLINE_1_HEADING = Math.PI;
    public static double SPLINE_1_TANGENT = 0;

    public static double STRAFE_1_VECTOR_X = 50;
    public static double STRAFE_1_VECTOR_Y = -60;

    public static double WAIT_SECONDS_2 = 2;
    SlidesWithActionsForAutos slides;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new SlidesWithActionsForAutos(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                // SPIKE MARK **MIDDLE**
                .lineToY(LINE_TO_Y_1_Y)

                .waitSeconds(WAIT_SECONDS_1)

                // AJUSTARE sa nu dai in PIXELUL MOV

                .lineToYConstantHeading(LINE_TO_Y_2_Y)

                //.lineToYSplineHeading(LINE_TO_Y_3_Y,LINE_TO_Y_3_HEADING)

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)

                //.splineToConstantHeading(new Vector2d(SPLINE_1_X, SPLINE_1_Y), SPLINE_1_TANGENT) // Spline catre Backdrop 1 tangent 0
                // deploy pixel 1
                .waitSeconds(WAIT_SECONDS_2)

                .strafeTo(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y))

                .build();
        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}
