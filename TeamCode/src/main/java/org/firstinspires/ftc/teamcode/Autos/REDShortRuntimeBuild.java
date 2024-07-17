package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
// NOT RECOMMENDED SINCE BUILDING TRAJECTORIES TAKES TIME, but IDK how to implement actions in a trajectory /////////////////////////

/*
* ┏┓┓ ┓   ┏┳┓┳┓┏┓┏┳┏┓┏┓┏┳┓┏┓┳┓┳┏┓┏┓  ┏┓┳┓┏┓  ┳┓┳┳┳┓ ┏┳┓  ┏┓┏┳┓  ┳┓┳┳┳┓┏┳┓┳┳┳┓┏┓
* ┣┫┃ ┃    ┃ ┣┫┣┫ ┃┣ ┃  ┃ ┃┃┣┫┃┣ ┗┓  ┣┫┣┫┣   ┣┫┃┃┃┃  ┃   ┣┫ ┃   ┣┫┃┃┃┃ ┃ ┃┃┃┃┣
* ┛┗┗┛┗┛   ┻ ┛┗┛┗┗┛┗┛┗┛ ┻ ┗┛┛┗┻┗┛┗┛  ┛┗┛┗┗┛  ┻┛┗┛┻┗┛ ┻   ┛┗ ┻   ┛┗┗┛┛┗ ┻ ┻┛ ┗┗┛
*/
public class REDShortRuntimeBuild extends LinearOpMode{
    MasterWithActionsClass master;
    Red3BoxVisionProcessor REDvisionProcessor;
    MecanumDrive drive;
    Action To_Left_Spike;
    Action To_Deploy_Yellow_Left;
    Action To_Deploy_Yellow_Middle;
    Action To_Deploy_Yellow_Right;
    Action To_Right_Spike;
    Action To_Middle_Spike;
    Action To_Park_In_Corner;
    VisionPortal visionPortal;
    Red3BoxVisionProcessor.Selected CASE;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        REDvisionProcessor = new Red3BoxVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), REDvisionProcessor);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));


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
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        // SPIKE MARK **LEFT**
                                        .lineToY(-40.2)
                                        .splineToLinearHeading(new Pose2d(8.50,-36.2,2.6179938779914944),1)
                                                .build(),
                                master.intake.DEPLOY_1(),
                                UpdatePoseForAuto(),
                                new ParallelAction(
                                        master.Chech_If_Arm_And_Rotate_Are_At_Deploy_And_Put_Them_There_If_Not(),

                                        drive.actionBuilder(drive.pose)
                                                // Mergi putin in spate ca sa nu dai in pixelul MOV
                                                .strafeTo(new Vector2d(13,-36))  // 14 , -40
                                                // Spline catre Backdrop 1
                                                .splineToLinearHeading(new Pose2d(48.4,-30.4,0),0)
                                                .build()
                                ),
                                master.slides.FORAUTO_Move_To_LEVEL_1(),
                                master.intake.DEPLOY_2(),
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        .strafeTo(new Vector2d(43,-58))
                                        .build(),
                                master.Prep_For_TeleOp()
                        )
                );
                break;
            case MIDDLE:
                Actions.runBlocking(
                        new SequentialAction(
                                master.Prep_For_Purple(),
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        // SPIKE MARK **MIDDLE**
                                        .lineToY(-33.2)
                                        // AJUSTARE sa nu dai in PIXELUL MOV
                                        .lineToYConstantHeading(-37.7) // -40.2
                                        .lineToYSplineHeading(-42.2,0) //-42.2
                                        .build(),
                                master.intake.DEPLOY_1(),
                                UpdatePoseForAuto(),
                                new ParallelAction(
                                        master.Chech_If_Arm_And_Rotate_Are_At_Deploy_And_Put_Them_There_If_Not(),
                                        drive.actionBuilder(drive.pose)
                                                // Spline catre Backdrop 1
                                                .splineToConstantHeading(new Vector2d(48.4, -35.4), 0)
                                                // deploy pixel 1
                                                .waitSeconds(1)
                                                .build()
                                ),
                                master.slides.FORAUTO_Move_To_LEVEL_1(),
                                master.intake.DEPLOY_2(),
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        .strafeTo(new Vector2d(43,-58))
                                        .build(),
                                master.Prep_For_TeleOp()
                        )
                );
                break;
            case RIGHT:
                Actions.runBlocking(
                        new SequentialAction(
                                master.Prep_For_Purple(),
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        // SPIKE MARK **RIGHT**

                                        .strafeToConstantHeading(new Vector2d(22,-36.2))

                                        .strafeToConstantHeading(new Vector2d(22,-39.2))

                                        // Mergi putin in spate ca sa nu dai in pixelul MOV

                                        .strafeTo(new Vector2d(30,-45))

                                        .build(),
                                master.intake.DEPLOY_1(),
                                UpdatePoseForAuto(),
                                new ParallelAction(
                                        master.Chech_If_Arm_And_Rotate_Are_At_Deploy_And_Put_Them_There_If_Not(),
                                        drive.actionBuilder(drive.pose)
                                                // Spline catre Backdrop 1
                                                .splineToLinearHeading(new Pose2d(48.4,-30.4,0),0)
                                                // deploy pixel 1
                                                .waitSeconds(1)
                                                .build()
                                ),
                                master.slides.FORAUTO_Move_To_LEVEL_1(),
                                master.intake.DEPLOY_2(),
                                UpdatePoseForAuto(),
                                drive.actionBuilder(drive.pose)
                                        .strafeTo(new Vector2d(43,-58))
                                        .build(),
                                master.Prep_For_TeleOp()
                        )
                );
                break;
        }
    }
    public Action UpdatePoseForAuto(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.updatePoseEstimate();
                return false;
            }
        };
    }
}