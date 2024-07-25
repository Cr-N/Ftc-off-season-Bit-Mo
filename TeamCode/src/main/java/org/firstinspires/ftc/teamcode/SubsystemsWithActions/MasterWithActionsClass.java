package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MasterWithActionsClass {
    public ArmWithActions arm;
    public ColorSensorWithActions colorSensor;
    public IntakeWithActions intake;
    public RotateWithActions rotate;
    public SlidesWithActionsForAutos slides;

    LinearOpMode myOpmode;

    public MasterWithActionsClass(LinearOpMode opMode){
        myOpmode = opMode;
        arm = new ArmWithActions(myOpmode.hardwareMap);
        colorSensor = new ColorSensorWithActions(myOpmode.hardwareMap);
        intake = new IntakeWithActions(myOpmode.hardwareMap);
        rotate = new RotateWithActions(myOpmode.hardwareMap);
        slides = new SlidesWithActionsForAutos(myOpmode.hardwareMap);
    }
    public SequentialAction Intake_Then_prep_ROTATE_and_ARM_for_Deploy(){
        return new SequentialAction(
            slides.UNLOCK_SLIDES(),
            Chech_If_Arm_And_Rotate_Are_At_Deploy_And_Put_Them_There_If_Not(),
            slides.FORAUTO_Move_To_LEVEL_INTAKE_POSITION(),
            new ParallelAction(
                    arm.Arm_To_PickUp_Position(),
                    rotate.Rotate_To_Pick_Up_Position()
            ),
            new SleepAction(0.3),
            intake.GRAB(),
            arm.Arm_To_Deploy_Position(),
            rotate.Rotate_To_Deploy_Position()
        );
    }
    public SequentialAction Chech_If_Arm_And_Rotate_Are_At_Deploy_And_Put_Them_There_If_Not(){
        if(arm.Get_Arm_State() != ArmWithActions.Params.ArmStates.AT_DEPLOY_POSITION){
            return new SequentialAction(
                    arm.Arm_To_Deploy_Position(),
                    rotate.Rotate_To_Deploy_Position()
            );
        }
        else return new SequentialAction(
                new NullAction()
        );
    }
    public SequentialAction Prep_For_Purple(){
            return new SequentialAction(
              intake.PICKUP(),
              arm.Arm_To_Intermediary_Position(),
              rotate.Rotate_To_Pick_Up_Position(),
              new SleepAction(1.5),
              arm.Arm_To_PickUp_Position(),
              new SleepAction(1),
              intake.GRAB(),
              new SleepAction(2),
              arm.Arm_To_Purple_Pixel_Deploy_Positon(),
              rotate.Rotate_To_Purple_Pixel_Deploy_Position()
            );
    }
    public SequentialAction Score_Yellow(){
        return new SequentialAction(
          arm.Arm_To_Deploy_Position(),
          rotate.Rotate_To_Deploy_Position(),
                new SleepAction(1),
                slides.FORAUTO_Move_To_LEVEL_2(),
                new SleepAction(0.5),
                intake.DEPLOY_2(),
                new SleepAction(0.2),
          slides.FORAUTO_Move_To_LEVEL_1(),
          new SleepAction(0.5),
          slides.FORAUTO_Move_To_LEVEL_INTAKE_POSITION()
          );
    }
    public SequentialAction Prep_For_TeleOp(){
        return new SequentialAction(
                intake.PICKUP(),
                rotate.Rotate_To_Start_For_Tele(),
                arm.Arm_To_Intermediary_Position(),
                slides.FORAUTO_Move_To_LEVEL_INTAKE_POSITION()
        );
    }
}
