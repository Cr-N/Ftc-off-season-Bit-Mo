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
public class RED_SHORT_RIGHT extends LinearOpMode {

    public static double STRAFE_1_X = 22;
    public static double STRAFE_1_Y = -36.2;
    public static double STRAFE_2_X = 22;
    public static double STRAFE_2_Y = -39.2;
    public static double STRAFE_3_X = 30;
    public static double STRAFE_3_Y = -45;
    public static double SPLINE_1_X = 48.4;
    public static double SPLINE_1_Y = -30.4;
    public static double SPLINE_1_HEADING = 0;
    public static double SPLINE_1_TANGENT = 0;
    public static double PARK_X = 50;
    public static double PARK_Y = -60;
    SlidesWithActionsForAutos slides;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new SlidesWithActionsForAutos(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                // SPIKE MARK RIGHT
                .strafeToConstantHeading(new Vector2d(STRAFE_1_X,STRAFE_1_Y))

                .strafeToConstantHeading(new Vector2d(STRAFE_2_X,STRAFE_2_Y))

                // Mergi putin in spate ca sa nu dai in pixelul MOV
                .strafeTo(new Vector2d(STRAFE_3_X,STRAFE_3_Y))  // 14 , -40

                // Spline catre Backdrop 1

                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)

                // Park
                .strafeTo(new Vector2d(PARK_X,PARK_Y))
                .build();
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}
