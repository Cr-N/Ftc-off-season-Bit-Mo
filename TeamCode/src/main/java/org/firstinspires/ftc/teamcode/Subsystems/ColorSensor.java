package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;
// the sensor we have is a REV V3 Color + Distance Sensor, the data sheet form the REV website tells us that
// the distance measured by the sensor is between 1cm-10cm but I think we need to test that first to see if we need MM or just CM for the measuring unit
// Sensor Scale for HSV: 360 Degrees, [0,1] , [0,1]
public class ColorSensor implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    SpecialGamepad FeedBack_Gamepad;
    public static class Params{
        public double distance;
        public double min_distance_for_detection = 0.660; // in CM


        public enum DistanceStates{
            OBJECT_CLOSE,
            OBJECT_NOT_CLOSE
        }

    }
    public static Params PARAMETERS = new Params();
    Params.DistanceStates Current_DistanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;
    Params.DistanceStates Last_DistanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;

    private final NormalizedColorSensor colorSensor;
    public ColorSensor(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        FeedBack_Gamepad = new SpecialGamepad(myOpmode);

        if (this.hardwareMap != null) {
            colorSensor = myOpmode.hardwareMap.get(NormalizedColorSensor.class,"sensor_color");
            if(colorSensor instanceof DistanceSensor){
                PARAMETERS.distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            }
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void Update_Sensor_Data(){
        PARAMETERS.distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);
    }
    public void Handle_Pixel_Feedback(){
        Update_Sensor_Data();

        if(PARAMETERS.distance < PARAMETERS.min_distance_for_detection){
            Current_DistanceState = Params.DistanceStates.OBJECT_CLOSE;
            if( Current_DistanceState== Params.DistanceStates.OBJECT_CLOSE && Last_DistanceState == Params.DistanceStates.OBJECT_NOT_CLOSE)
                Handle_Driver_Feedback_For_Pixels();
        }
        else{
            Current_DistanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;
        }
        Last_DistanceState = Current_DistanceState;
    }
    public void Handle_Driver_Feedback_For_Pixels(){
        if(Current_DistanceState == Params.DistanceStates.OBJECT_CLOSE){
            FeedBack_Gamepad.Rumble_Quick_Blip();
            FeedBack_Gamepad.LED_Red_Alert();
        }
    }
    public Params.DistanceStates Get_Distance_State() {
        return Current_DistanceState;
    }
    public double Get_Current_Distance_Param(){
        return PARAMETERS.distance;
    }
    public double Get_Minimum_Detection_Distance_Parameter(){
        return PARAMETERS.min_distance_for_detection;
    }

}
