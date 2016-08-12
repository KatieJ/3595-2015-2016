package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by katiejohnson on 11/3/15.
 */

// NOTE: This is one of multiple GyroDemo op modes -- if it behaves strangley look into the other ones

public class GyroDemo extends LinearOpMode
{
    GyroSensor legacyGyro;
    double offset = 0; // was called gyroAverage when I first wrote code
    double error;
    double timeVariable;
    DcMotor leftMotor;
    DcMotor rightMotor;
    double motorPower = 0.3;
    double finalLeftMotorPower = 0;
    double finalRightMotorPower = 0;

    public void runOpMode() throws InterruptedException
    {
        legacyGyro = hardwareMap.gyroSensor.get("sensor_gyro"); // get ref to gyro
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitForStart();
        InitializeGyro();
        for (int i = 0; i < 1000000000; i++) // usually would be within while loop going for time/clicks
        {
            GyroCorrection (i);
            telemetry.addData("Error ", error);
        }
    }

    void InitializeGyro ()
    {
        int loopRuns = 200; // number of times for loop to run
        int gyroReadings = 0; // sum of readings

        for (int i = 0; i < loopRuns; i++)
        {
            gyroReadings += legacyGyro.getRotation();
        }
        offset = (double)gyroReadings/loopRuns;
    }

    void GyroCorrection (int iteration)
    {
        // Assuming that everything is in seconds

        if (iteration == 0)
        {
            error = 0;
            // also reset timer
            timeVariable = this.time;
        }

        error += (legacyGyro.getRotation()-offset) * (timeVariable - this.time) * 0.09;
        telemetry.addData("Gyro rate", legacyGyro.getRotation()-offset);

        if (motorPower - error > 1.0)
            finalLeftMotorPower = 1.0;
        else if (motorPower - error < -1.0)
            finalLeftMotorPower = -1.0;
        else
            finalLeftMotorPower = motorPower - error;

        if (motorPower + error < -1.0)
            finalRightMotorPower = -1.0;
        else if (motorPower + error > 1.0)
            finalRightMotorPower = 1.0;
        else
            finalRightMotorPower = motorPower + error;

        leftMotor.setPower(finalLeftMotorPower); // assuming subtraction for left
        rightMotor.setPower(finalRightMotorPower); // assuming addition for right

        timeVariable = this.time;
    }
}
