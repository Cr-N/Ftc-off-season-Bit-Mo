package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SpecialGamepad{
    boolean lastA=false,currentA,lastY=false,currentY,lastB=false,currentB,lastX=false,currentX,lastDpadUp=false,currentDpadUp,lastDpadDown=false,currentDpadDown,lastDpadRight=false,currentDpadRight,lastDpadLeft=false,currentDpadLeft,lastRightStickButton=false,currentRightStickButton,lastLeftStickButton=false,currentLeftStickButton,lastStartButton=false,currentStartButton,lastBackButton=false,currentBackButton,lastRB=false,currentRB,lastLB=false,currentLB;
    GamepadEx SpecialGamepadForCheckingButtonPresses;
    LinearOpMode myOpmode = null;
    Gamepad.RumbleEffect quickBlip;
    Gamepad.LedEffect whiteAlert;
    public SpecialGamepad(LinearOpMode opMode) {
        myOpmode = opMode;
        this.SpecialGamepadForCheckingButtonPresses = new GamepadEx(myOpmode.gamepad1);
    }
    public void updateCurrentStates(){
        currentA= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.A);
        currentY= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.Y);
        currentB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.B);
        currentX= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.X);
        currentDpadUp= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_UP);
        currentDpadDown= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_DOWN);
        currentDpadRight= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_RIGHT);
        currentDpadLeft= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_LEFT);
        currentRightStickButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        currentLeftStickButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON);
        currentStartButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.START);
        currentBackButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.BACK);
        currentRB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        currentLB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.LEFT_BUMPER);
    }
    public void updateLastStates(){
        lastA= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.A);
        lastY= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.Y);
        lastB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.B);
        lastX= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.X);
        lastDpadUp= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_UP);
        lastDpadDown= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_DOWN);
        lastDpadRight= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_RIGHT);
        lastDpadLeft= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.DPAD_LEFT);
        lastRightStickButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        lastLeftStickButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON);
        lastStartButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.START);
        lastBackButton= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.BACK);
        lastRB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        lastLB= SpecialGamepadForCheckingButtonPresses.isDown(GamepadKeys.Button.LEFT_BUMPER);
    }
    public boolean isPressed_Button_A(){
        return (currentA && !lastA);
    }
    public boolean isPressed_Button_X(){
        return (currentX && !lastX);
    }
    public boolean isPressed_Button_B(){
        return (currentB && !lastB);
    }
    public boolean isPressed_Button_Y(){
        return (currentY && !lastY);
    }
    public boolean isPressed_Button_Dpad_Up(){
        return (currentDpadUp && !lastDpadUp);
    }
    public boolean isPressed_Button_Dpad_Down(){
        return (currentDpadDown && !lastDpadDown);
    }
    public boolean isPressed_Button_Dpad_Left(){
        return (currentDpadLeft && !lastDpadLeft);
    }
    public boolean isPressed_Button_Dpad_Right(){
        return (currentDpadRight && !lastDpadRight);
    }
    public boolean isPressed_Button_BACK(){
        return (currentBackButton && !lastBackButton);
    }
    public boolean isPressed_Button_START(){
        return (currentStartButton && !lastStartButton);
    }
    public boolean isPressed_Button_Right_Stick_Button(){
        return (currentRightStickButton && !lastRightStickButton);
    }
    public boolean isPressed_Button_Left_Stick_Button(){
        return (currentLeftStickButton && !lastLeftStickButton);
    }
    public boolean isPressed_Button_Left_Bumper(){
        return (currentLB && !lastLB);
    }
    public boolean isPressed_Button_Right_Bumper(){
        return (currentRB && !lastRB);
    }
    public void Rumble_Quick_Blip(){
       quickBlip = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5,0,150)
                .addStep(0,0.5,150)
                .addStep(1,1,150)
               .build();
       myOpmode.gamepad1.runRumbleEffect(quickBlip);
    }
    public void LED_White_Alert(){
        whiteAlert = new Gamepad.LedEffect.Builder()
                .addStep(255,255,255,25)
                .addStep(0,0,0,25)
                .addStep(255,255,255,25)
                .addStep(0,0,0,25)
                .addStep(255,255,255,25)
                .addStep(0,0,0,25)
                .addStep(255,255,255,25)
                .addStep(0,0,0,25)
                .build();
        myOpmode.gamepad1.runLedEffect(whiteAlert);
    }
    public void LED_Yellow_Alert(){
        whiteAlert = new Gamepad.LedEffect.Builder()
                .addStep(255,255,0,25)
                .addStep(0,0,0,225)
                .addStep(255,255,0,25)
                .addStep(0,0,0,25)
                .addStep(255,255,0,25)
                .addStep(0,0,0,25)
                .addStep(255,255,0,25)
                .addStep(0,0,0,25)
                .build();
        myOpmode.gamepad1.runLedEffect(whiteAlert);
    }
    public void LED_Green_Alert(){
        whiteAlert = new Gamepad.LedEffect.Builder()
                .addStep(0,255,0,25)
                .addStep(0,0,0,25)
                .addStep(0,255,0,25)
                .addStep(0,0,0,25)
                .addStep(0,255,0,25)
                .addStep(0,0,0,25)
                .addStep(0,255,0,25)
                .addStep(0,0,0,25)
                .build();
        myOpmode.gamepad1.runLedEffect(whiteAlert);
    }
    public void LED_Purple_Alert(){
        whiteAlert = new Gamepad.LedEffect.Builder()
                .addStep(119,0,200,25)
                .addStep(0,0,0,25)
                .addStep(119,0,200,25)
                .addStep(0,0,0,25)
                .addStep(119,0,200,25)
                .addStep(0,0,0,25)
                .addStep(119,0,200,25)
                .addStep(0,0,0,25)
                .build();
        myOpmode.gamepad1.runLedEffect(whiteAlert);
    }
}
