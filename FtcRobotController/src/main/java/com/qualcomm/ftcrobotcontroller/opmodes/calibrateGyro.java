package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by katiejohnson on 11/2/15.
 */

// Class to initialise integrating gyro's x axis before use
// Takes 200 readings from gyro sensor and averages them to get offset

public class calibrateGyro extends LinearOpMode
{
    GyroSensor integratingGyro;
    double gyroReadings;
    double offset;
    int iterations = 200; // number of times for loop to run

    public void runOpMode() throws InterruptedException // copied from PushBotSquare
    {
        waitForStart();
        integratingGyro = hardwareMap.gyroSensor.get("integratingGyro"); // Get ref to gyro

        for(int i = 0; i < iterations; i++)
        {
            //read from gyro and add reading to "gyro readings"
            gyroReadings += integratingGyro.getHeading();
        }
        offset = gyroReadings/iterations; // take average of gyro readings
        telemetry.addData("Gyro average (on x axis)", offset);
    }
}