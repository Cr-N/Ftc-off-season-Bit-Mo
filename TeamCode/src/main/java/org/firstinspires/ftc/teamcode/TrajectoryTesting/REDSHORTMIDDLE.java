package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;

@Config
@Autonomous
public class REDSHORTMIDDLE extends LinearOpMode {
    MasterWithActionsClass master;
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
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                .afterTime(0.1,master.Prep_For_Purple())
                .waitSeconds(5)
                // SPIKE MARK **MIDDLE**
                .lineToY(LINE_TO_Y_1_Y)

                .waitSeconds(WAIT_SECONDS_1)

                .lineToYConstantHeading(LINE_TO_Y_2_Y)

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)
                //.afterTime(0.2,master.Score_Yellow())
                .waitSeconds(WAIT_SECONDS_2)

                .strafeTo(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y))
                //.stopAndAdd(master.Prep_For_TeleOp())
                .build();
        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}
