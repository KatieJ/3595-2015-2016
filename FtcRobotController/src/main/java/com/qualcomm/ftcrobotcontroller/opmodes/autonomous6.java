// Code works to score to climbers in shelter, drive to corner of field, wait for autonomous to almost end, then drive into station
// It is untuned, and it was written with the shelter shifted too far away
// Left as of March 19, 2016


package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;


public class autonomous6 extends LinearOpMode {
    // Declaring motors, servos, and sensors
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftBrushMotor, rightBrushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo;
    GyroSensor integratingGyro;

    public ElapsedTime driveTimer = new ElapsedTime();
    public ElapsedTime autoTimer = new ElapsedTime();

    int i = 0; // iterations of while loop
    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;
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
        getReference();

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

        // So that robot is maneuverable during set-up
        leftDrive1.setPowerFloat();
        leftDrive2.setPowerFloat();
        rightDrive1.setPowerFloat();
        rightDrive2.setPowerFloat();

        waitForStart(); // Wait for op mode to be started

        autoTimer.reset();

        calibrateGyro(); // move this before waitForStart() and reset z axis integrator or what it is here

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);

        // Drive away from wall
        gyroCorrectionBackward(-0.35, 2250, 2);

        // Turn towards shelter
        turnCounterClockwise(0.75, 46);

        // Drive to shelter
        gyroCorrectionBackward(-0.3, 11400, 5.5);
        // start slowly so that it doesn't pop a wheelie

        // Stop brush
        leftBrushMotor.setPower(0.0);
        rightBrushMotor.setPower(0.0);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftAutoOut);
        rightAutoServo.setPosition(rightAutoOut);
        sleep(2000); // Wait for servos to extend

        // Retract servos
        leftAutoServo.setPosition(leftAutoIn);
        rightAutoServo.setPosition(rightAutoIn);
        sleep(1000); // Wait for servos to retract (play with this time)

        // Back away from shelter (so that robot doesn't catch when turning)
        gyroCorrectionForward(0.3, 750, 1.5);

        turnClockwise(.65, 46);
        gyroCorrectionBackward(-0.4, 16000, 3.0);

        while (autoTimer.time() < 26) {
            telemetry.addData("while loop running", i);
            sleep(50);
        }
        gyroCorrectionForward(1.0, 5000, 3.0); // clicks are so big because I want it to time out instead of going for distance (because I'm pushing another robot probably)
    }

    // Calculate error and adjust motor powers accordingly
    void gyroCorrectionForward(double motorPower, int clicks, double timeout)
            throws InterruptedException
    {
        double loopSpeed;
        double previousTime = 0.0;
        double currentTime;
        driveTimer.reset();
        integratingGyro.resetZAxisIntegrator();
        int backupTime = 200;
        double xDegreesTurned = 0.0;
        double error;
        double gain = 0.05; // gain to multiply error by
        while (leftDrive1.getCurrentPosition() < clicks && driveTimer.time() < timeout) {
            currentTime = driveTimer.time();
            loopSpeed = currentTime - previousTime;
            previousTime = currentTime;

            xDegreesTurned += (integratingGyro.rawX() - offset) * loopSpeed;

            error = headingShift(integratingGyro.getHeading());

            if (xDegreesTurned > 250) { // If robot is on debris, back up
                leftMotorPower = -motorPower;
                rightMotorPower = -motorPower;
            } else { // Otherwise adjust motor power based on error
                leftMotorPower = motorPower - error * gain; // Adjust left motor power
                rightMotorPower = motorPower + error * gain; // Adjust right motor power
            }

            // Set motor powers
            leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
            leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
            rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0));
            rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0));

            // If robot is on debris, wait while it backs up
            if (xDegreesTurned > 250) {
                sleep(backupTime);
                // If robot has to back up, add to time to back up for next time
                backupTime += 200; // This might cause issues (also play with *= 2)
            }
        }
        // Brake drive motors when method is done
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }

    // Calculate error and adjust motor powers accordingly
    void gyroCorrectionBackward(double motorPower, int clicks, double timeout) // attach note to make power negative
            throws InterruptedException
    {
        double loopSpeed;
        double previousTime = 0.0;
        double currentTime;
        driveTimer.reset();
        integratingGyro.resetZAxisIntegrator();
        //motorPower = -motorPower;
        int backupTime = 200;
        double xDegreesTurned = 0.0;
        double error;
        double gain = 0.05; // gain to multiply error by
        while (leftDrive1.getCurrentPosition() > -clicks && driveTimer.time() < timeout) {
            currentTime = driveTimer.time();
            loopSpeed = currentTime - previousTime;
            previousTime = currentTime;

            xDegreesTurned += (integratingGyro.rawX() - offset) * loopSpeed;

            error = headingShift(integratingGyro.getHeading());

            if (xDegreesTurned > 250) { // If robot is on debris, back up
                leftMotorPower = -motorPower;
                rightMotorPower = -motorPower;
            } else { // Otherwise adjust motor power based on error
                leftMotorPower = motorPower - error * gain; // Adjust left motor power
                rightMotorPower = motorPower + error * gain; // Adjust right motor power
            }

            // Set motor powers
            leftDrive1.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
            leftDrive2.setPower(Range.clip(leftMotorPower, -1.0, 1.0));
            rightDrive1.setPower(Range.clip(rightMotorPower, -1.0, 1.0));
            rightDrive2.setPower(Range.clip(rightMotorPower, -1.0, 1.0));

            // If robot is on debris, wait while it backs up
            if (xDegreesTurned > 250) {
                sleep(backupTime);
                // If robot has to back up, add to time to back up for next time
                backupTime += 200;
            }
        }
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }

    // Turn clockwise with gyro
    void turnClockwise (double motorPower, int degrees) {
        while (headingShift(integratingGyro.getHeading()) < degrees) {
            leftDrive1.setPower(motorPower);
            leftDrive2.setPower(motorPower);
            rightDrive1.setPower(-motorPower);
            rightDrive2.setPower(-motorPower);
        }
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }

    // Turn counter clockwise with gyro
    void turnCounterClockwise (double motorPower, int degrees) {
        while (headingShift(integratingGyro.getHeading()) > -degrees) {
            leftDrive1.setPower(-motorPower);
            leftDrive2.setPower(-motorPower);
            rightDrive1.setPower(motorPower);
            rightDrive2.setPower(motorPower);
        }
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }

    // Heading returns a number from 0 to 360, adjust so that -180 to 180 is returned
    double headingShift (double heading) {
        return ((heading+180)%360)-180;
    }

    void calibrateGyro() throws InterruptedException {
        // Calibrate gyro (x and z axes) and ensure that it's finished
        integratingGyro.calibrate();
        Thread.sleep(100);
        while (integratingGyro.isCalibrating() || i < 200){
            gyroReadings += integratingGyro.rawX();
            ++i;
        }
        offset = gyroReadings/i; // take average of gyro readings on x axis
    }

    void getReference() {
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
    }
}