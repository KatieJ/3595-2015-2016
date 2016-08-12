package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Autonomous program (used at State Championship)
// Drive forward with gyro correction, score two climbers in shelter, end in beacon repair zone

public class autonomous3 extends LinearOpMode {
    // Declaring motors, servos, and sensors
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, brushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo;
    GyroSensor integratingGyro;

    public ElapsedTime timer = new ElapsedTime();

    double error = 0.0;
    double currentHeading = 0.0;
    double leftMotorPower = 0.0; // Adjusted left motor power
    double rightMotorPower = 0.0; // Adjusted right motor power

    // Positions for different servos
    double rightLiftUp = 0.0;
    double leftLiftUp = 1.0;
    double leftAutoIn = 0.0;
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
        brushMotor = hardwareMap.dcMotor.get("brushMotor");

        // Reverse right drive motors and brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        brushMotor.setDirection(DcMotor.Direction.REVERSE);

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

        integratingGyro.calibrate(); // Calibrate the gyro

        waitForStart(); // Wait for op mode to be started

        // Make sure gyro is calibrated
        while (integratingGyro.isCalibrating())
            Thread.sleep(50);

        brushMotor.setPower(-1.0); // Reverse brush to clear debris

        timer.reset();

        // Drive forward with gyro correction
        while (timer.time() < 4.5)
            gyroCorrection(-0.35);
        // Brake motors
        brushMotor.setPower(0.0);
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftAutoOut); // Extend left servo
        rightAutoServo.setPosition(rightAutoOut); // Extend right servo
        sleep(1000); // Wait for autonomous servos to fully extend
        leftAutoServo.setPosition(leftAutoIn); // Retract left servo
        rightAutoServo.setPosition(rightAutoIn); // Retract right servo
    }

    void gyroCorrection(double motorPower) // Calculate error and adjust motor powers accordingly
    {
        double gain = 0.05; // gain to multiply error by

        currentHeading = integratingGyro.getHeading(); // So that program only has to "getHeading" once
        if (currentHeading >= 180)
            error = currentHeading - 360;
        else
            error = currentHeading;

        leftMotorPower = motorPower - error * gain; // Adjust left motor power
        rightMotorPower = motorPower + error * gain; // Adjust right motor power

        leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0)); // Set left motor power
        leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0)); // Set left motor power
        rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0)); // Set right motor power
        rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0)); // Set right motor power
    }
}