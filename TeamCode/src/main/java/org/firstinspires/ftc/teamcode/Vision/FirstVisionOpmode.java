package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.teamcode.Vision.FirstVisionProcessor;
 import org.firstinspires.ftc.vision.VisionPortal;


public class FirstVisionOpmode extends OpMode {
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    @Override
    public void init() {
             visionProcessor = new FirstVisionProcessor();
             visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
             visionPortal.stopStreaming();
    }
    @Override
     public void loop() {
                 telemetry.addData("Identified", visionProcessor.getSelection());
    }
}