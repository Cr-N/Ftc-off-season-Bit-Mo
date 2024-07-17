package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
import org.firstinspires.ftc.teamcode.Vision.Red3BoxVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
// Refer to .stopAndAdd question thread on FTC Discord -> Roadrunner help
// This will use afterTime directly in the trajectory in order to implement parallel actions while robot is moving
/*
 * ┏┓┓ ┓   ┳┳┓┏┓┓┏┏┓┳┳┓┏┓┳┓┏┳┓  ┳┏┓  ┏┓┏┓┏┓┏┓┳┓┏┓┏┳┓┏┓┳┓  ┏┓┏┓┏┓┳┓┏┓┏┓┏┓┓┏
 * ┣┫┃ ┃   ┃┃┃┃┃┃┃┣ ┃┃┃┣ ┃┃ ┃   ┃┗┓  ┗┓┣ ┃┃┣ ┣┫┣┫ ┃ ┣ ┃┃  ┣┫┃┃┃┃┣┫┃┃┣┫┃ ┣┫
 * ┛┗┗┛┗┛  ┛ ┗┗┛┗┛┗┛┛ ┗┗┛┛┗ ┻   ┻┗┛  ┗┛┗┛┣┛┗┛┛┗┛┗ ┻ ┗┛┻┛  ┛┗┣┛┣┛┛┗┗┛┛┗┗┛┛┗
 */
public class REDShortAFTERTIME extends LinearOpMode{
    MasterWithActionsClass master;
    Red3BoxVisionProcessor REDvisionProcessor;
    MecanumDrive drive;
    Action LEFT_TRAJ , RIGHT_TRAJ , MIDDLE_TRAJ;
    VisionPortal visionPortal;
    Red3BoxVisionProcessor.Selected CASE;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        REDvisionProcessor = new Red3BoxVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), REDvisionProcessor);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        // left case trajectory
        {
            LEFT_TRAJ = drive.actionBuilder(drive.pose)
                    // SPIKE MARK **LEFT**
                    .lineToY(-40.2)

                    .splineToLinearHeading(new Pose2d(8.50,-36.2,2.6179938779914944),1)

                    .afterTime(0.2,master.intake.DEPLOY_1())
                    .waitSeconds(2)

                    // Mergi putin in spate ca sa nu dai in pixelul MOV
                    .strafeTo(new Vector2d(13,-36))

                    // Spline catre Backdrop 1
                    .splineToLinearHeading(new Pose2d(48.4,-30.4,0),0)

                    .afterTime(0.2,master.Score_Yellow())
                    .waitSeconds(3)

                    // Parcare
                    .strafeTo(new Vector2d(43,-58))
                    .build();

        }
        // middle case trajectory
        {
            MIDDLE_TRAJ = drive.actionBuilder(drive.pose)

                        // SPIKE MARK **MIDDLE**
                        .lineToY(-33.2)

                        .afterTime(0.2,master.intake.DEPLOY_1())
                        .waitSeconds(2)

                        // AJUSTARE sa nu dai in PIXELUL MOV
                        .lineToYConstantHeading(-37.7)

                        .lineToYSplineHeading(-42.2,0)

                        // Spline catre Backdrop 1
                        .splineToConstantHeading(new Vector2d(48.4, -35.4), 0)

                        // deploy pixel 1
                        .afterTime(0.2,master.Score_Yellow())
                        .waitSeconds(3)

                        .strafeTo(new Vector2d(43,-58))
                        .build();
        }
        // right case trajectory
        {
            RIGHT_TRAJ = drive.actionBuilder(drive.pose)
                    // SPIKE MARK **RIGHT**

                    .strafeToConstantHeading(new Vector2d(22,-36.2))

                    .strafeToConstantHeading(new Vector2d(22,-39.2))

                    .afterTime(0.2,master.intake.DEPLOY_1())
                    .waitSeconds(2)

                    // Mergi putin in spate ca sa nu dai in pixelul MOV

                    .strafeTo(new Vector2d(30,-45))

                    // Spline catre Backdrop 1
                    .splineToLinearHeading(new Pose2d(48.4,-30.4,0),0)

                    // deploy pixel 1
                    .afterTime(0.2,master.Score_Yellow())
                    .waitSeconds(3)

                    .strafeTo(new Vector2d(43,-58))

                    .build();
        }
        while (opModeInInit() && !isStopRequested()){

            switch (REDvisionProcessor.getSelection()){
                case LEFT:
                    telemetry.addLine(" ┏┓┏┓┏┓┏┓   ┓  ┏┓ ┏┓ ┏┳┓ ");
                    telemetry.addLine(" ┃ ┣┫┗┓┣  • ┃  ┣  ┣   ┃  ");
                    telemetry.addLine(" ┗┛┛┗┗┛┗┛ • ┗┛ ┗┛ ┻   ┻  ");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.LEFT;
                    break;
                case RIGHT:
                    telemetry.addLine(" ┏┓┏┓┏┓┏┓   ┳┓ ┳ ┏┓ ┓┏ ┏┳┓ ");
                    telemetry.addLine(" ┃ ┣┫┗┓┣  • ┣┫ ┃ ┃┓ ┣┫  ┃  ");
                    telemetry.addLine(" ┗┛┛┗┗┛┗┛ • ┛┗ ┻ ┗┛ ┛┗  ┻  ");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.LEFT;

                    break;
                case MIDDLE:
                    telemetry.addLine("┏┓┏┓┏┓┏┓   ┳┳┓ ┳ ┳┓ ┳┓ ┓  ┏┓ ");
                    telemetry.addLine("┃ ┣┫┗┓┣  • ┃┃┃ ┃ ┃┃ ┃┃ ┃  ┣  ");
                    telemetry.addLine("┗┛┛┗┗┛┗┛ • ┛ ┗ ┻ ┻┛ ┻┛ ┗┛ ┗┛ ");
                    telemetry.update();

                    CASE = Red3BoxVisionProcessor.Selected.LEFT;

                    break;
                case NONE:
                    telemetry.addLine("┏┓┏┓┏┓┏┓   ┳┓ ┏┓ ┳┓ ┏┓ ");
                    telemetry.addLine("┃ ┣┫┗┓┣  • ┃┃ ┃┃ ┃┃ ┣  ");
                    telemetry.addLine("┗┛┛┗┗┛┗┛ • ┛┗ ┗┛ ┛┗ ┗┛ ");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.RIGHT;
                    break;
            }
        }
        if(isStopRequested())return;
        switch (CASE){
            case LEFT:
                Actions.runBlocking(
                        new SequentialAction(
                                master.Prep_For_Purple(),
                                LEFT_TRAJ,
                                master.Prep_For_TeleOp()
                        )
                );
                break;
            case MIDDLE:
                Actions.runBlocking(
                        new SequentialAction(
                                master.Prep_For_Purple(),
                                MIDDLE_TRAJ,
                                master.Prep_For_TeleOp()
                        )
                );
                break;
            case RIGHT:
                Actions.runBlocking(
                        new SequentialAction(
                                master.Prep_For_Purple(),
                                RIGHT_TRAJ,
                                master.Prep_For_TeleOp()
                        )
                );
                break;
        }
    }
}