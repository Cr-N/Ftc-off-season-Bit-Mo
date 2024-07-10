package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MasterClass {
    public Arm Arm_Subsystem;
    public Slides Slides_subsystem;
    public Rotate Rotate_subsystem;
    public Intake Claw_subsystem;
    public ColorSensor Color_sensor_subsystem;
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    public MasterClass(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        Arm_Subsystem = new Arm(myOpmode);
        Slides_subsystem = new Slides(myOpmode);
        Rotate_subsystem = new Rotate(myOpmode);
        Color_sensor_subsystem = new ColorSensor(myOpmode);
        Claw_subsystem = new Intake(myOpmode);
    }

    public void handleLoweringArm(){

        switch (Arm_Subsystem.getArmState()){
            case AT_DEPLOY_POSITION:
                Slides_subsystem.changeControlStateTo_LEVELS_MODE();
                Slides_subsystem.moveToLEVEL_IntakePosition();
                Slides_subsystem.LOCK_SLIDES();
                Arm_Subsystem.Arm_To_IntermediaryPosition();
                Rotate_subsystem.Rotate_To_Pick_Up_Position();
                break;
            case AT_INTERMEDIARY_POSITION:
                if(Slides_subsystem.getControlState() == Slides.Params.ControlState.MANUAL_MODE){
                    Slides_subsystem.changeControlStateTo_LEVELS_MODE();
                }
                if(Slides_subsystem.getStateofSlides() == Slides.Params.State_of_slides.INTAKE_POSITION)
                    Arm_Subsystem.Arm_To_Pick_Up_Position();
                else{
                    Slides_subsystem.moveToLEVEL_IntakePosition();
                    Arm_Subsystem.Arm_To_Pick_Up_Position();
                }
                Slides_subsystem.LOCK_SLIDES();
                break;
            case AT_PICK_UP_POSITION:
                Slides_subsystem.changeControlStateTo_LEVELS_MODE();
                Slides_subsystem.moveToLEVEL_IntakePosition();
                Arm_Subsystem.Arm_To_Pick_Up_Position();
                Rotate_subsystem.Rotate_To_Pick_Up_Position();
                Arm_Subsystem.ArmState = Arm.Params.ArmStates.AT_INTERMEDIARY_POSITION;
        }
    }
    public void handle_slide_levels_UP(){
            if(Slides_subsystem.controlState == Slides.Params.ControlState.LEVELS_MODE)
            {
                switch (Slides_subsystem.getStateofSlides()) {
                    case INTAKE_POSITION:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_1();
                        break;
                    case AT_LEVEL_1:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_2();
                        break;
                    case AT_LEVEL_2:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_3();
                        break;
                    case AT_LEVEL_3:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_4();
                        break;

                }
            }
    }
    public void handle_slide_levels_DOWN(){

            if(Slides_subsystem.controlState == Slides.Params.ControlState.LEVELS_MODE)
            {
                switch (Slides_subsystem.getStateofSlides()) {
                    case AT_LEVEL_1:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_IntakePosition();
                        break;
                    case AT_LEVEL_2:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_1();
                        break;
                    case AT_LEVEL_3:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_2();
                        break;
                    case AT_LEVEL_4:
                        if(Arm_Subsystem.getArmState() == Arm.Params.ArmStates.AT_DEPLOY_POSITION)
                            Slides_subsystem.moveToLEVEL_3();
                        break;
                }
            }
    }
}
