package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// this subsystem is NOT FOR TELEOP, only use this in autos or where we don't need anything to do with gamepad
public class ColorSensorWithActions{
    public static class Params{
        public double distance;
        public double min_distance_for_detection = 0.660; // in CM

        public enum DistanceStates{
            OBJECT_CLOSE,
            OBJECT_NOT_CLOSE
        }

    }

    public static Params PARAMETERS = new Params();

    Params.DistanceStates distanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;


    private final NormalizedColorSensor colorSensor;

    public ColorSensorWithActions(HardwareMap hardwareMap) {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"sensor_color");
        PARAMETERS.distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

    public void Update_Sensor_Data(){
        PARAMETERS.distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);
    }

    /**
     * This action will update the current detection state(close or not close) and will end immediately after being run
     */
    public class Get_Sensor_Detection implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            PARAMETERS.distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);

            telemetryPacket.put("object state ",distanceState);

            if(PARAMETERS.distance < PARAMETERS.min_distance_for_detection){
                distanceState = Params.DistanceStates.OBJECT_CLOSE;
            }
            else{
                distanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;
            }
            return false;
        }
    }

    /**
     * This will <span style="color:red;">halt</span> the program until we get an object detection from the color/distance sensor
     */
    public class HALT_PROGRAM_UNTIL_DETECTION_USE_WITH_CARE implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            PARAMETERS.distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);

            telemetryPacket.put("object state ",distanceState);

            if(PARAMETERS.distance < PARAMETERS.min_distance_for_detection){
                distanceState = Params.DistanceStates.OBJECT_CLOSE;
                return false;
            }
            else{
                distanceState = Params.DistanceStates.OBJECT_NOT_CLOSE;
                return true;
            }
        }
    }

    public Action Get_Sensor_Detection(){
        return new Get_Sensor_Detection();
    }

    public Action HALT_PROGRAM_UNTIL_DETECTION_USE_WITH_CARE(){
        return new HALT_PROGRAM_UNTIL_DETECTION_USE_WITH_CARE();
    }

    public Params.DistanceStates Get_Distance_State() {
        return distanceState;
    }
    public double Get_Current_Distance_Param(){
        return PARAMETERS.distance;
    }
    public double Get_Minimum_Detection_Distance_Parameter(){
        return PARAMETERS.min_distance_for_detection;
    }

}
