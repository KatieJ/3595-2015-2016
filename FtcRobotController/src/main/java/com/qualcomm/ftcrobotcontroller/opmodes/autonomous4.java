// A test autonomous from March 15 -- it works, but I moved on to autonomous 5 to try anti-tip when running into debris

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;


public class autonomous4 extends LinearOpMode {
    // Declaring motors, servos, and sensors
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftBrushMotor, rightBrushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo;
    GyroSensor integratingGyro;

    public ElapsedTime timer = new ElapsedTime();

    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;

    // Positions for different servos
    double rightLiftUp = 0.0;
    double leftLiftUp = 1.0;
    double leftAutoIn = 0.02;
    double leftAutoOut = 1.0;
    double rightAutoIn = 0.95;
    double rightAutoOut = 0.0;
    double leftDebrisIn = 0.424;
    double rightDebrisIn = 1.0;
    double releaseLift = 1.0;
    double rightZipIn = 1.0;
    double leftZipIn = 0.0;
    double toggleClose = 0.824;

    public void runOpMode() throws InterruptedException {
        // Get references to motors
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        leftBrushMotor = hardwareMap.dcMotor.get("leftBrushMotor");
        rightBrushMotor = hardwareMap.dcMotor.get("rightBrushMotor");

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get references to servos
        leftLiftServo = hardwareMap.servo.get("leftLiftServo");
        rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        leftAutoServo = hardwareMap.servo.get("leftAutoServo");
        rightAutoServo = hardwareMap.servo.get("rightAutoServo");
        brakeServo = hardwareMap.servo.get("brakeServo");
        leftDebrisServo = hardwareMap.servo.get("leftDebrisServo");
        rightDebrisServo = hardwareMap.servo.get("rightDebrisServo");
        rightZipServo = hardwareMap.servo.get("rightZipServo");
        leftZipServo = hardwareMap.servo.get("leftZipServo");
        toggleServo = hardwareMap.servo.get("toggleServo");

        // Get reference to gyro
        integratingGyro = hardwareMap.gyroSensor.get("integratingGyro");

        // Initialize servos
        brakeServo.setPosition(releaseLift);
        rightZipServo.setPosition(rightZipIn);
        leftZipServo.setPosition(leftZipIn);
        leftDebrisServo.setPosition(leftDebrisIn);
        rightDebrisServo.setPosition(rightDebrisIn);
        leftLiftServo.setPosition(leftLiftUp);
        rightLiftServo.setPosition(rightLiftUp);
        rightAutoServo.setPosition(rightAutoIn);
        leftAutoServo.setPosition(leftAutoIn);
        toggleServo.setPosition(toggleClose);

        waitForStart(); // Wait for op mode to be started

        // Calibrate gyro and make sure it's finished
        integratingGyro.calibrate();
        do
            Thread.sleep(50);
        while (integratingGyro.isCalibrating());

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);

        timer.reset();

        // Drive forward with gyro correction
        integratingGyro.resetZAxisIntegrator();
        while (leftDrive1.getCurrentPosition() > -2150)
            gyroCorrection(-0.35);

        // turn with gyro
        integratingGyro.resetZAxisIntegrator();
        while (((integratingGyro.getHeading()+180)%360)-180 > -45) {
            leftDrive1.setPower(-0.65);
            leftDrive2.setPower(-0.65);
            rightDrive1.setPower(0.65);
            rightDrive2.setPower(0.65);
        }
        // Should make ((num+180)%360 - 180 ) a method

        // Drive forward with gyro correction
        timer.reset();
        integratingGyro.resetZAxisIntegrator();
        while (leftDrive1.getCurrentPosition() > -11500 && timer.time() < 4.5)
            gyroCorrection(-0.4);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftAutoOut); // Extend left servo
        rightAutoServo.setPosition(rightAutoOut); // Extend right servo
        sleep(1250); // Wait for autonomous servos to fully extend
        leftAutoServo.setPosition(leftAutoIn); // Retract left servo
        rightAutoServo.setPosition(rightAutoIn); // Retract right servo

        // Brake motors - there's a bug where motors don't necessarily stop when told, loop makes this less of an issues
        for(int i = 0; i < 10; i++) {
            leftBrushMotor.setPower(0.0);
            rightBrushMotor.setPower(0.0);
            leftDrive1.setPower(0.0);
            leftDrive2.setPower(0.0);
            rightDrive1.setPower(0.0);
            rightDrive2.setPower(0.0);
        }

        // Set motors to float so robot can be moved more easily - remove before competition
        leftBrushMotor.setPowerFloat();
        rightBrushMotor.setPowerFloat();
        leftDrive1.setPowerFloat();
        leftDrive2.setPowerFloat();
        rightDrive1.setPowerFloat();
        rightDrive2.setPowerFloat();
    }

    void gyroCorrection(double motorPower) // Calculate error and adjust motor powers accordingly
    {
        double currentHeading; // So program only has to "getHeading" once
        double error;
        double gain = 0.05; // gain to multiply error by

        currentHeading = integratingGyro.getHeading();
        if (currentHeading >= 180)
            error = currentHeading - 360;
        else
            error = currentHeading;

        leftMotorPower = motorPower - error * gain; // Adjust left motor power
        rightMotorPower = motorPower + error * gain; // Adjust right motor power

        // Set motor powers
        leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
        leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
        rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0));
        rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0));
    }
}