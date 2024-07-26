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
public class BLUE_SHORT_RIGHT extends LinearOpMode {

    double RIGHT_LINE_TO_Y_1_Y = 46;
    double RIGHT_SPLINE_1_X = 15;
    double RIGHT_SPLINE_1_Y = 38;
    double RIGHT_SPLINE_1_HEADING = 2.09+(Math.PI/2);
    double RIGHT_SPLINE_1_TANGENT = 1;
    double RIGHT_WAIT_SECONDS_1 = 1;
    double RIGHT_STRAFE_1_VECTOR_X = 15;
    double RIGHT_STRAFE_1_VECTOR_Y = 39;
    double RIGHT_SPLINE_2_X = 53;
    double RIGHT_SPLINE_2_Y = 18;
    double RIGHT_SPLINE_2_HEADING = Math.PI;
    double RIGHT_SPLINE_2_TANGENT = 0;
    double RIGHT_WAIT_SECONDS_2 = 3.5;
    double RIGHT_PARK_X = 53;
    double RIGHT_PARK_Y = 58;
    double RIGHT_PARK_TANGENT = 0;

    MasterWithActionsClass master;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, 63.2, 3*Math.PI/2));
        master = new MasterWithActionsClass(this);
        traj = drive.actionBuilder(drive.pose)
                // SPIKE MARK **LEFT**
                .lineToY(RIGHT_LINE_TO_Y_1_Y)

                .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_1_X,RIGHT_SPLINE_1_Y,RIGHT_SPLINE_1_HEADING),RIGHT_SPLINE_1_TANGENT)
                .afterTime(0.6 ,master.intake.DEPLOY_1())
                .waitSeconds(RIGHT_WAIT_SECONDS_1)

                // Mergi putin in spate ca sa nu dai in pixelul MOV
                //.strafeTo(new Vector2d(RIGHT_STRAFE_1_VECTOR_X,RIGHT_STRAFE_1_VECTOR_Y))

                // Spline catre Backdrop 1
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_2_X,RIGHT_SPLINE_2_Y,RIGHT_SPLINE_2_HEADING),RIGHT_SPLINE_2_TANGENT)
                .afterTime(0.1,master.Score_Yellow())
                .waitSeconds(RIGHT_WAIT_SECONDS_2)

                // Parcare
                .splineToConstantHeading(new Vector2d(RIGHT_PARK_X,RIGHT_PARK_Y),RIGHT_PARK_TANGENT)
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
