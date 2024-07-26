package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
@Disabled
@Config
@Autonomous
public class RED_SHORT_LEFT extends LinearOpMode {

    double LEFT_LINE_TO_Y_1_Y = -40.2;
    double LEFT_SPLINE_1_X = 7.5;
    double LEFT_SPLINE_1_Y = -40;
    double LEFT_SPLINE_1_HEADING = 2.09;
    double LEFT_SPLINE_1_TANGENT = 1;
    double LEFT_WAIT_SECONDS_1 = 1;
    double LEFT_STRAFE_1_VECTOR_X = 15;
    double LEFT_STRAFE_1_VECTOR_Y = -39;
    double LEFT_SPLINE_2_X = 59.4;
    double LEFT_SPLINE_2_Y = -24;
    double LEFT_SPLINE_2_HEADING = Math.PI;
    double LEFT_SPLINE_2_TANGENT = 0;
    double LEFT_WAIT_SECONDS_2 = 2;
    double LEFT_PARK_X = 53;
    double LEFT_PARK_Y = -60;
    double LEFT_PARK_TANGENT = 0;

    MasterWithActionsClass master;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        master = new MasterWithActionsClass(this);
        traj = drive.actionBuilder(drive.pose)
                // SPIKE MARK **LEFT**
                .lineToY(LEFT_LINE_TO_Y_1_Y)

                .splineToLinearHeading(new Pose2d(LEFT_SPLINE_1_X,LEFT_SPLINE_1_Y,LEFT_SPLINE_1_HEADING),LEFT_SPLINE_1_TANGENT)
                .afterTime(0.6 ,master.intake.DEPLOY_1())
                .waitSeconds(LEFT_WAIT_SECONDS_1)

                // Mergi putin in spate ca sa nu dai in pixelul MOV
                .strafeTo(new Vector2d(LEFT_STRAFE_1_VECTOR_X,LEFT_STRAFE_1_VECTOR_Y))

                // Spline catre Backdrop 1
                .splineToLinearHeading(new Pose2d(LEFT_SPLINE_2_X,LEFT_SPLINE_2_Y,LEFT_SPLINE_2_HEADING),LEFT_SPLINE_2_TANGENT)
                .afterTime(0.1,master.Score_Yellow())
                .waitSeconds(LEFT_WAIT_SECONDS_1)

                // Parcare
                .splineToConstantHeading(new Vector2d(LEFT_PARK_X,LEFT_PARK_Y),LEFT_PARK_TANGENT)
                .build();

        Actions.runBlocking(
                master.Prep_For_Purple()
        );
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}
