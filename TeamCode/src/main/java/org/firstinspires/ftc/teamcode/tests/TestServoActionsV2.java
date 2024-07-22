package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
@Config
@TeleOp
public class TestServoActionsV2 extends OpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    public SimpleServo servo;
    public static double servoError =0.05;
    public static double low =0;
    public static double high =180;
    boolean currentA;
    boolean lastA = false;
    @Override
    public void init() {
        servo = new SimpleServo(hardwareMap,"myServo",0,180, AngleUnit.DEGREES);
        }

    @Override
    public void loop() {
        double time = getRuntime();
        TelemetryPacket packet = new TelemetryPacket();
        currentA= gamepad1.a;
        // updated based on gamepads
        if(currentA && !lastA){
            runningActions.add(new SequentialAction(
                    MoveToLow(),
                    TeleMetryActionForTesting(),
                    MoveToHigh(),
                    TeleMetryActionForTesting(),
                    MoveToLow(),
                    TeleMetryActionForTesting(),
                    MoveToHigh()
            ));
        }


        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
        lastA=currentA;
        telemetry.addData("loop time ",time - getRuntime());
        telemetry.update();
    }

    public class MoveToLow implements Action {
        private boolean initialized1 = false;
        private double servoAngle;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized1 == false){
                servo.turnToAngle(low);
                initialized1 = true;
            }
            servoAngle = servo.getAngle();
            telemetryPacket.put("angle: ", servoAngle);
            telemetryPacket.put("target - current angle : ", low-servoAngle);

            telemetry.addLine("Running to LOW");
            return servoAngle > low;
        }
    }


    public class TeleMetryActionForTesting implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addLine("WE ARE RUNNING A INBETWEEN TELEMETRY TEST!");
            return false;
        }
    }
    public Action TeleMetryActionForTesting(){
        return new TeleMetryActionForTesting();
    }
    public class MoveToHigh implements Action {
        private boolean initialized2 = false;
        private double servoAngle;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (initialized2 == false) {
                servo.turnToAngle(high);
                initialized2 = true;
            }
            servoAngle = servo.getAngle();
            telemetryPacket.put("angle: ", servoAngle);
            telemetryPacket.put("target - current angle : ", high - servoAngle);
            telemetry.addLine("Running to HIGH");
            return servoAngle < high;
        }
    }
    public Action MoveToLow() {
        return new SequentialAction(
                new SleepAction(1),
                new InstantAction(() -> servo.setPosition(low))
        );
    }

    public Action MoveToHigh() {
        return new SequentialAction(
                new SleepAction(1),
                new InstantAction(() -> servo.setPosition(high))
        );
    }
}