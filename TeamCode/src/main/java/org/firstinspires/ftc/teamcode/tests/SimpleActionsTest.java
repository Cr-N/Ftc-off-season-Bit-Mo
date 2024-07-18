package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SimpleActionsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry , FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("In init");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        FIRSTAction(),
                        SECONDAction()
                )
        );
    }
    public class FIRSTAction implements Action{
        private boolean initialized = false;
        double time1,time2;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized == false){
                time1 = getRuntime();
                initialized = true;
            }
            time2 =getRuntime();
            telemetryPacket.addLine("This is the first action!");
            telemetryPacket.put("Time left for action",time1+5.00 - time2);
            if(time2 >=time1+5.00)
            {
                return false;
            }
            else
            return true;
        }
    }
    public class SECONDAction implements Action{
        private boolean initialized = false;
        double time1,time2;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized == false){
                time1 = getRuntime();
                initialized = true;
            }
            time2 =getRuntime();
            telemetryPacket.addLine("This is the second action!");
            telemetryPacket.put("Time left for action",time1+5.00 - time2);
            if(time2 >=time1+5.00)
                return false;
            else
                return true;
        }
    }
    public Action FIRSTAction(){
        return new FIRSTAction();
    }
    public Action SECONDAction(){
        return new SECONDAction();
    }
}
