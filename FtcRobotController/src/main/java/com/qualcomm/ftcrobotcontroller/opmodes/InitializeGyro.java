package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by katiejohnson on 11/2/15.
 */

// Class to initialise gyro sensor before use
// Takes 200 readings from gyro sensor and averages them to get offset

// THIS IS FOR THE LEGACY GYRO
// IF I WANT TO CALIBRATE/INITIALIZE THE INTEGRATING GYRO, THEN USE "calibrateGyro"

public class InitializeGyro extends LinearOpMode
{
    GyroSensor legacyGyro;
    double gyroReadings;
    double gyroAverage;
    int loopRuns = 200; // number of times for loop to run

    public void runOpMode() throws InterruptedException // copied from PushBotSquare
    {
        waitForStart();
        legacyGyro = hardwareMap.gyroSensor.get("sensor_gyro"); // Get ref to gyro

        for(int i = 0; i < loopRuns; i++)
        {
            //read from gyro and add reading to "gyro readings"
            gyroReadings += legacyGyro.getRotation();
        }
        gyroAverage = gyroReadings/loopRuns; // take average of gyro readings
        telemetry.addData("Gyro average", gyroAverage);
    }
}