package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp
public class ServoTestingforIntake extends LinearOpMode{
    private Intake intake;
    boolean lastLB = false, currentLB , lastRB = false , currentRB , lastRight_Stick_Button = false , currentRight_Stick_Button;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        GamepadEx gm1 = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()){
        currentLB = gm1.isDown(GamepadKeys.Button.LEFT_BUMPER);
        currentRB = gm1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        currentRight_Stick_Button = gm1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        if(currentLB && !lastLB){
            intake.DEPLOY_1();
        }
        if(currentRB && !lastRB){
            intake.DEPLOY_2();
        }
        if(currentRight_Stick_Button && !lastRight_Stick_Button){
            intake.PICKUP();
        }

           telemetry.addData("s", intake.Get_Intake_State());
            telemetry.update();

            lastLB = currentLB;
        lastRB = currentRB;
        lastRight_Stick_Button = currentRight_Stick_Button;
        }

    }
}
