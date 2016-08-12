package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;


public class autonomous5 extends LinearOpMode {
    // Declaring motors, servos, and sensors
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftBrushMotor, rightBrushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo;
    GyroSensor integratingGyro;

    public ElapsedTime timer = new ElapsedTime();

    int i = 0; // iterations of while loop
    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;
    double xDegreesTurned = 0.0;
    double loopSpeed = 0.0;
    double previousTime = 0.0;
    double currentTime = 0.0;
    double offset = 0.0;
    double gyroReadings = 0.0;

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

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

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

        // Calibrate gyro (x and z axes) and ensure that it's finished
        integratingGyro.calibrate();
        Thread.sleep(100);
        while (integratingGyro.isCalibrating() || i < 200){
            gyroReadings += integratingGyro.rawX();
            ++i;
        }
        offset = gyroReadings/i; // take average of gyro readings on x axis

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);

        // Drive forward with gyro correction
        timer.reset();
        integratingGyro.resetZAxisIntegrator();
        //while (leftDrive1.getCurrentPosition() > -2150)
        while(leftDrive1.getCurrentPosition() > -2250 && timer.time() < 2)
            gyroCorrection(-0.35);

        // Shouldn't need to reset z axis
        integratingGyro.resetZAxisIntegrator();
        turnCounterClockwise(0.65, 45);

        // Drive forward with gyro correction
        timer.reset();
        integratingGyro.resetZAxisIntegrator();
        while (leftDrive1.getCurrentPosition() > -11400 && timer.time() < 4.5)
            gyroCorrection(-0.4);

        // Brake motors
        leftBrushMotor.setPower(0.0);
        rightBrushMotor.setPower(0.0);
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftAutoOut);
        rightAutoServo.setPosition(rightAutoOut);
        sleep(2000); // Wait for servos to extend

        // Retract servos
        leftAutoServo.setPosition(leftAutoIn);
        rightAutoServo.setPosition(rightAutoIn);
        sleep(1000); // Wait for servos to retract before stopping program

        leftDrive1.setPowerFloat();
        leftDrive2.setPowerFloat();
        rightDrive1.setPowerFloat();
        rightDrive2.setPowerFloat();
    }

    // Maybe move while loop into here, parameters should be timeout, power, and clicks
    void gyroCorrection(double motorPower) throws InterruptedException // Calculate error and adjust motor powers accordingly
    {
        double error;
        double gain = 0.05; // gain to multiply error by
        currentTime = timer.time();
        loopSpeed = currentTime - previousTime;
        previousTime = currentTime;

        xDegreesTurned += (integratingGyro.rawX()-offset)*loopSpeed;

        error = headingShift(integratingGyro.getHeading());

        // If robot has tilted back, then back up
        if(xDegreesTurned > 250) {
            //leftMotorPower = -motorPower - error * gain; // Adjust left motor power
            //rightMotorPower = -motorPower + error * gain; // Adjust right motor power
            // Trying it without gyro correction because I added sleep
            leftMotorPower = -motorPower;
            rightMotorPower = -motorPower;
        } else {
            leftMotorPower = motorPower - error * gain; // Adjust left motor power
            rightMotorPower = motorPower + error * gain; // Adjust right motor power
        }

        // Set motor powers
        leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
        leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
        rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0));
        rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0));

        // If robot was on debris, back up for 200 ms
        if(xDegreesTurned > 250)
            sleep(200);
    }

    void turnClockwise (double motorPower, int degrees) {
        while (headingShift(integratingGyro.getHeading()) < degrees) {
            leftDrive1.setPower(motorPower);
            leftDrive2.setPower(motorPower);
            rightDrive1.setPower(-motorPower);
            rightDrive2.setPower(-motorPower);
        }
    }

    void turnCounterClockwise (double motorPower, int degrees) {
        while (headingShift(integratingGyro.getHeading()) > -degrees) {
            leftDrive1.setPower(-motorPower);
            leftDrive2.setPower(-motorPower);
            rightDrive1.setPower(motorPower);
            rightDrive2.setPower(motorPower);
        }
    }

    // Heading returns a number from 0 to 360, adjust so that -180 to 180 is returned
    double headingShift (double heading) {
        return ((heading+180)%360)-180;
    }
}