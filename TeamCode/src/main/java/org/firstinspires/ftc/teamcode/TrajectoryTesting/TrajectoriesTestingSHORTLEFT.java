package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class TrajectoriesTestingSHORTLEFT extends LinearOpMode {

    public static double LINE_TO_Y_1_Y = -40.2;
    public static double SPLINE_1_X = 6;
    public static double SPLINE_1_Y = -33.2;
    public static double SPLINE_1_HEADING = 2.09;
    public static double SPLINE_1_TANGENT = 1;
    public static double WAIT_SECONDS_1 = 2;
    public static double STRAFE_1_VECTOR_X = 13;
    public static double STRAFE_1_VECTOR_Y = -36;
    public static double SPLINE_2_X = 50;
    public static double SPLINE_2_Y = -26;
    public static double SPLINE_2_HEADING = Math.PI;
    public static double SPLINE_2_TANGENT = 0;
    public static double WAIT_SECONDS_2 = 3;
    public static double STRAFE_2_VECTOR_X = 46;
    public static double STRAFE_2_VECTOR_Y = -60;






    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                // SPIKE MARK **LEFT**
                .lineToY(LINE_TO_Y_1_Y)

                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)

                .waitSeconds(WAIT_SECONDS_1)

                // Mergi putin in spate ca sa nu dai in pixelul MOV
                .strafeTo(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y))

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(SPLINE_2_X,SPLINE_2_Y,SPLINE_2_HEADING),SPLINE_2_TANGENT)

                .waitSeconds(WAIT_SECONDS_2)

                // Parcare
                .strafeTo(new Vector2d(STRAFE_2_VECTOR_X,STRAFE_2_VECTOR_Y))
                .build();
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}
