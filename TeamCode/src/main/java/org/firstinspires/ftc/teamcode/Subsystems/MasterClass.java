package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardWare.DriveBase;

public class MasterClass {
    public Arm Arm_Subsystem;
    public Slides Slides_subsystem;
    public Rotate Rotate_subsystem;
    public Intake Claw_subsystem;
    public ColorSensor Color_sensor_subsystem;
    public DriveBase driveBase;
    public GamepadEx TRIGGERGAMEPAD;
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    public MasterClass(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        TRIGGERGAMEPAD = new GamepadEx(myOpmode.gamepad1);
        Arm_Subsystem = new Arm(myOpmode);
        Slides_subsystem = new Slides(myOpmode);
        Rotate_subsystem = new Rotate(myOpmode);
        Color_sensor_subsystem = new ColorSensor(myOpmode);
        Claw_subsystem = new Intake(myOpmode);
        driveBase = new DriveBase(myOpmode);
    }

    public void handleLoweringArm(){

        switch (Arm_Subsystem.Get_Arm_State()){
            case AT_DEPLOY_POSITION:
                Slides_subsystem.Change_CONTROL_STATE_To_LEVELS_MODE();
                Slides_subsystem.Move_To_LEVEL_INTAKE_POSITION();
                Slides_subsystem.LOCK_SLIDES();
                Arm_Subsystem.Arm_To_Intermediary_Position();
                Rotate_subsystem.Rotate_To_Pick_Up_Position();
                break;
            case AT_INTERMEDIARY_POSITION:
                if(Slides_subsystem.Get_Control_State() == Slides.Params.ControlState.MANUAL_MODE){
                    Slides_subsystem.Change_CONTROL_STATE_To_LEVELS_MODE();
                }
                if(Slides_subsystem.Get_Left_Pos() <25 && Slides_subsystem.Get_Right_Pos() < 25)
                    Arm_Subsystem.Arm_To_PickUp_Position();
                else{
                    Slides_subsystem.Move_To_LEVEL_INTAKE_POSITION();
                }
                Slides_subsystem.LOCK_SLIDES();
                break;
            case AT_PICK_UP_POSITION:
                Slides_subsystem.Change_CONTROL_STATE_To_LEVELS_MODE();
                Slides_subsystem.Move_To_LEVEL_INTAKE_POSITION();
                Arm_Subsystem.Arm_To_PickUp_Position();
                Rotate_subsystem.Rotate_To_Pick_Up_Position();
                Arm_Subsystem.ArmState = Arm.Params.ArmStates.AT_INTERMEDIARY_POSITION;
                break;
        }
    }
    public void handle_slide_levels_UP(){
            if(Slides_subsystem.controlState == Slides.Params.ControlState.LEVELS_MODE)
            {
                switch (Slides_subsystem.Get_State_of_Slides()) {
                    case INTAKE_POSITION:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_1();
                        break;
                    case AT_LEVEL_1:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To__LEVEL_2();
                        break;
                    case AT_LEVEL_2:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_3();
                        break;
                    case AT_LEVEL_3:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_4();
                        break;

                }
            }
    }
    public void handle_slide_levels_DOWN(){

            if(Slides_subsystem.controlState == Slides.Params.ControlState.LEVELS_MODE)
            {
                switch (Slides_subsystem.Get_State_of_Slides()) {
                    case AT_LEVEL_1:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_INTAKE_POSITION();
                        break;
                    case AT_LEVEL_2:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_1();
                        break;
                    case AT_LEVEL_3:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To__LEVEL_2();
                        break;
                    case AT_LEVEL_4:
                        if(Arm_Subsystem.Get_Arm_State() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.Move_To_LEVEL_3();
                        break;
                }
            }
    }
    public void Handle_Arm_Up(){
        switch (Arm_Subsystem.Get_Arm_State()){
            case AT_PICK_UP_POSITION:
                Arm_Subsystem.Arm_To_Intermediary_Position();
                Slides_subsystem.LOCK_SLIDES();
                break;
            case AT_INTERMEDIARY_POSITION:
                Arm_Subsystem.Arm_To_Deploy_Position();
                Rotate_subsystem.Rotate_To_Deploy_Position();
                Slides_subsystem.UNLOCK_SLIDES();
                break;

        }
    }
}
