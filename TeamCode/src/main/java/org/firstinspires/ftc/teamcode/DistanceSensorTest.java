package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends OpMode {

    // Define the Digital Channel device
    private DigitalChannel beamBreakSensor;

    @Override
    public void init() {
        beamBreakSensor = hardwareMap.get(DigitalChannel.class, "distance_sensor");

        // Set the pin mode to INPUT to read data from the sensor
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
    }

    boolean previousState = false;
    int counter = 0;
    @Override
    public void loop() {
        // In Digital mode: returns true if an object is within the dial's set distance
        // Returns false if the path is clear
        boolean objectDetected = beamBreakSensor.getState();
        if(objectDetected && !previousState) counter += 1;
        previousState = objectDetected;
        if(gamepad1.b) counter = 0;

        // Output to Driver Station
        telemetry.addData("Blocked", objectDetected ? "Yes" : "No");
        telemetry.addData("Counter", counter);
        telemetry.addData("Press B to", "reset");
        telemetry.update();
    }
}