package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class IntegratingGyroDemo extends LinearOpMode
{
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2; // Used Dc motors
    TouchSensor touchSensor;

    // Declaring variables
    GyroSensor integratingGyro;
    double error = 0;
    double currentHeading = 0;
    double leftMotorPower = 0; // Adjusted left motor power
    double rightMotorPower = 0; // Adjusted right motor power
    double motorPower = 0.0; // Standard motor power

    public void runOpMode() throws InterruptedException
    {
        integratingGyro = hardwareMap.gyroSensor.get("integratingGyro"); // Get ref to gyro
        //Get references to motors
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        // Reverse right motors so that all motors go in same direction when given positive power
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        integratingGyro.calibrate(); // calibrate the gyro
        boolean pressed = touchSensor.isPressed();

        waitForStart(); // Wait for op mode to be completely started

        // Can potentially go before waitForStart();
        // code in while loop from Modern Robotics
        // make sure the gyro is calibrated
        while (integratingGyro.isCalibrating())
        {
            Thread.sleep(50);
        }

        while(touchSensor.isPressed() == pressed)
        {
            GyroCorrection(); // Call gyro correction method
            telemetry.addData("Error", error); // Display gyro heading
        }
    }

    void GyroCorrection() // Calculate error and adjust motor powers accordingly
    {
        double gain = 0.05; // gain to multiply error by

        currentHeading = integratingGyro.getHeading(); // So that program only has to "getHeading" once
        if (currentHeading >= 180)
        {
            error = currentHeading - 360;
        }
        else
        {
            error = currentHeading;
        }

        leftMotorPower = motorPower - error*gain; // Adjust left motor power
        rightMotorPower = motorPower + error*gain; // Adjust right motor power

        leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0)); // Set left motor power
        leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0)); // Set left motor power
        rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0)); // Set right motor power
        rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0)); // Set right motor power
    }
}