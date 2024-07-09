package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;

public class ColorSensor implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    SpecialGamepad FeedBack_Gamepad;
    public static class Params{
        public NormalizedRGBA colorsRGBA;
        public double distance;
        public double min_distance_for_detection = 100; // in MM
        public float gain = 2;
        public float[] HSVforDetection;
        //TODO: tune the detection values accordingly
        public float Min_RED_Color_Value_For_WHITE_Detection = 0.90F;
        public float Min_GREEN_Color_Value_For_WHITE_Detection = 0.90F;
        public float Min_BLUE_Color_Value_For_WHITE_Detection = 0.90F;
        public float[] White_HSV_Detection_MIN  = {0,4,93};
        public float[] White_HSV_Detection_MAX  = {360,0,100};
        public float[] Green_HSV_Detection_MIN  = {100,69,51};
        public float[] Green_HSV_Detection_MAX  = {138, 100, 100};
        public float[] Yellow_HSV_Detection_MIN = {42,79,46}; //54,76,40
        public float[] Yellow_HSV_Detection_MAX = {68,100,100}; //54,93,100
        public float[] Purple_HSV_Detection_MIN = {265,59,32};//252, 67, 54
        public float[] Purple_HSV_Detection_MAX = {288,100,100};//252,51,98
        public enum DetectionStates{
                DETECTED_NOTHING,
                DETECTED_WHITE,
                DETECTED_YELLOW,
                DETECTED_PURPLE,
                DETECTED_GREEN
        };
        public enum DistanceStates{
            OBJECT_CLOSE,
            OBJECT_NOT_CLOSE
        }

    }
    public static Params PARAMETERS = new Params();
    Params.DetectionStates DetectionState = Params.DetectionStates.DETECTED_NOTHING;
    Params.DistanceStates DistanceState = Params.DistanceStates.OBJECT_CLOSE;
    private final NormalizedColorSensor colorSensor;
    public ColorSensor(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        FeedBack_Gamepad = new SpecialGamepad(myOpmode);

        if (this.hardwareMap != null) {
            colorSensor = myOpmode.hardwareMap.get(NormalizedColorSensor.class,"sensor_color");
            if(colorSensor instanceof DistanceSensor){
                PARAMETERS.distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
            }
            if(colorSensor instanceof SwitchableLight){
                ((SwitchableLight) colorSensor).enableLight(true);
            }
            colorSensor.setGain(PARAMETERS.gain);
            PARAMETERS.colorsRGBA = colorSensor.getNormalizedColors();

        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void updateSensorData(){
        PARAMETERS.distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.MM);
        PARAMETERS.colorsRGBA = colorSensor.getNormalizedColors();
        Color.colorToHSV(PARAMETERS.colorsRGBA.toColor(),PARAMETERS.HSVforDetection);
    }
    public void handleColorDetections(){
        if(PARAMETERS.colorsRGBA.red > PARAMETERS.Min_RED_Color_Value_For_WHITE_Detection && PARAMETERS.colorsRGBA.blue > PARAMETERS.Min_GREEN_Color_Value_For_WHITE_Detection && PARAMETERS.colorsRGBA.green > PARAMETERS.Min_BLUE_Color_Value_For_WHITE_Detection){
            DetectionState = Params.DetectionStates.DETECTED_WHITE;
        }
        else if(PARAMETERS.HSVforDetection[0] >= PARAMETERS.Yellow_HSV_Detection_MIN[0] && PARAMETERS.HSVforDetection[1] >= PARAMETERS.Yellow_HSV_Detection_MIN[1] && PARAMETERS.HSVforDetection[2] >= PARAMETERS.Yellow_HSV_Detection_MIN[2] && PARAMETERS.HSVforDetection[0] <= PARAMETERS.Yellow_HSV_Detection_MAX[0] && PARAMETERS.HSVforDetection[1] <= PARAMETERS.Yellow_HSV_Detection_MAX[1] && PARAMETERS.HSVforDetection[2] <= PARAMETERS.Yellow_HSV_Detection_MAX[2]){
            DetectionState = Params.DetectionStates.DETECTED_YELLOW;
        }
        else if(PARAMETERS.HSVforDetection[0] >= PARAMETERS.Green_HSV_Detection_MIN[0] && PARAMETERS.HSVforDetection[1] >= PARAMETERS.Green_HSV_Detection_MIN[1] && PARAMETERS.HSVforDetection[2] >= PARAMETERS.Green_HSV_Detection_MIN[2] && PARAMETERS.HSVforDetection[0] <= PARAMETERS.Green_HSV_Detection_MAX[0] && PARAMETERS.HSVforDetection[1] <= PARAMETERS.Green_HSV_Detection_MAX[1] && PARAMETERS.HSVforDetection[2] <= PARAMETERS.Green_HSV_Detection_MAX[2]){
            DetectionState = Params.DetectionStates.DETECTED_GREEN;
        }
        else if(PARAMETERS.HSVforDetection[0] >= PARAMETERS.Purple_HSV_Detection_MIN[0] && PARAMETERS.HSVforDetection[1] >= PARAMETERS.Purple_HSV_Detection_MIN[1] && PARAMETERS.HSVforDetection[2] >= PARAMETERS.Purple_HSV_Detection_MIN[2] && PARAMETERS.HSVforDetection[0] <= PARAMETERS.Purple_HSV_Detection_MAX[0] && PARAMETERS.HSVforDetection[1] <= PARAMETERS.Purple_HSV_Detection_MAX[1] && PARAMETERS.HSVforDetection[2] <= PARAMETERS.Purple_HSV_Detection_MAX[2]){
            DetectionState = Params.DetectionStates.DETECTED_PURPLE;
        }
        else{
            DetectionState = Params.DetectionStates.DETECTED_NOTHING;

        }
    }
    public void handleDistanceDetections(){
        if(PARAMETERS.distance <PARAMETERS.min_distance_for_detection){
            DistanceState = Params.DistanceStates.OBJECT_CLOSE;
        }
        else{
            DistanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;
        }
    }
    public void handle_Driver_Feedback_For_Pixels(){
        if(DistanceState == Params.DistanceStates.OBJECT_CLOSE){
            if(DetectionState == Params.DetectionStates.DETECTED_WHITE){
                FeedBack_Gamepad.Rumble_Quick_Blip();
                FeedBack_Gamepad.LED_White_Alert();
            }
            if(DetectionState == Params.DetectionStates.DETECTED_GREEN){
                FeedBack_Gamepad.Rumble_Quick_Blip();
                FeedBack_Gamepad.LED_Green_Alert();
            }
            if(DetectionState == Params.DetectionStates.DETECTED_YELLOW){
                FeedBack_Gamepad.Rumble_Quick_Blip();
                FeedBack_Gamepad.LED_Yellow_Alert();
            }
            if(DetectionState == Params.DetectionStates.DETECTED_PURPLE){
                FeedBack_Gamepad.Rumble_Quick_Blip();
                FeedBack_Gamepad.LED_Purple_Alert();
            }
        }
    }
    public Params.DetectionStates getDetectionState() {
        return DetectionState;
    }

    public Params.DistanceStates getDistanceState() {
        return DistanceState;
    }

}
