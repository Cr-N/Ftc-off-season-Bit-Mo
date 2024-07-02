package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardWare.Hardware;

@TeleOp(name = "TEST servo pentru intake")
public class ServoTestingforIntake extends LinearOpMode {
    GamepadEx gm1 = new GamepadEx(gamepad1);
    int servoPosition= 0;
    Hardware hardware = new Hardware(this);
    @Override
    public void runOpMode() {
        hardware.init();

        waitForStart();
        while(opModeIsActive()){

            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                if(servoPosition != 360) {
                    servoPosition = servoPosition + 10;
                    hardware.claw.turnToAngle(servoPosition);
                }
            }
            if(gm1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                if(servoPosition != 0) {
                    servoPosition = servoPosition - 10;
                    hardware.claw.turnToAngle(servoPosition);
                }
            }
        }

    }

}
