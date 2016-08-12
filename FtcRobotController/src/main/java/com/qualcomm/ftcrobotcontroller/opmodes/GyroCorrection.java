package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by katiejohnson on 11/2/15.
 */
public class GyroCorrection extends InitializeGyro // currently extends init gyro
{
    // currently assuming timer and gyro rate are in seconds not milliseconds
    ElapsedTime timer;
    double loopSpeed = 0; // setting equal to 0 might be problem when called >once
    double previousTime = 0;
    double gyroRotation;
    double degreesTurned;

    public void runOpMode() throws InterruptedException
    {
        loopSpeed = this.time-previousTime;
        previousTime = this.time;


        // might need to reset timer here or somewhere
        waitForStart();
        timer.reset();

        legacyGyro = hardwareMap.gyroSensor.get("sensor_gyro"); // Get ref to gyro
        gyroRotation = legacyGyro.getRotation();
        loopSpeed = previousTime; // - timer.time();
        //previousTime = timer.time(); // might cause issues cause few milliseconds off

        degreesTurned += loopSpeed*gyroRotation;
        telemetry.addData("Rotation", degreesTurned);
    }
}