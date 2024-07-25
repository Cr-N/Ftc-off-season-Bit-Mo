package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardWare.Hardware;
//
// https://www.youtube.com/watch?v=gnSW2QpkGXQ
//
@TeleOp(name = "Holonomic Drive" , group = "TEST")
public class Holonomic_Driving_Teleop extends LinearOpMode {
    Hardware hardware = new Hardware(this);

    double x,y,turn,theta,power,sin,cos,max,FLpow,FRpow,BLpow,BRpow;

    @Override
    public void runOpMode() {
        GamepadEx gm1 = new GamepadEx(gamepad1);
        hardware.init();
        waitForStart();
        while (opModeIsActive())
        {
            x = gm1.getLeftX(); // left stick x
            y = -gm1.getLeftY(); // left stick y
            turn = gm1.getRightX(); // right stick x

            theta = Math.atan2(y,x);
            power = Math.hypot(x,y);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max( Math.abs(sin) , Math.abs(cos) );

            FLpow = power * cos/max + turn;
            FRpow = power * sin/max - turn;
            BLpow = power * sin/max + turn;
            BRpow = power * cos/max - turn;

            if((power + Math.abs(turn)) > 1){
                FLpow /= power + Math.abs(turn);
                FRpow /= power + Math.abs(turn);
                BLpow /= power + Math.abs(turn);
                BRpow /= power + Math.abs(turn);
            }

            hardware.FL.setPower(FLpow);
            hardware.FR.setPower(FRpow);
            hardware.BL.setPower(BLpow);
            hardware.BR.setPower(BRpow);
        }

    }
}
