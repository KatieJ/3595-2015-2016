package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

public class BlueAutoFarMountWait extends LinearOpMode {
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
    double leftAutoIn = 0.06;
    double leftAutoOut = 1.0;
    double rightAutoIn = 0.95;
    double leftDebrisIn = 0.424;
    double rightDebrisIn = 1.0;
    double releaseLift = 1.0;
    double rightZipIn = 1.0;
    double leftZipIn = 0.0;
    double toggleClose = 0.45;

    public void runOpMode() throws InterruptedException {
        getReference();

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        leftDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightDrive1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightDrive2.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

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

        calibrateGyro();

        waitForStart(); // Wait for op mode to be started
        autoTimer.reset();
        sleep(10000);

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);

        // Drive away from wall
        gyroCorrection(-0.35, 800, 1.5);

        // Turn towards shelter
        turnClockwise(0.75, 50);

        // Drive to shelter
        gyroCorrection(-0.35, 13000, 8.5);

        // Stop brush
        leftBrushMotor.setPower(0.0);
        rightBrushMotor.setPower(0.0);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftAutoOut);
        sleep(2000); // Wait for servo to extend

        // Retract servo
        leftAutoServo.setPosition(leftAutoIn);
        sleep(1000);
    }

    // Calculate error and adjust motor powers accordingly
    void gyroCorrection(double motorPower, int clicks, double timeout)
            throws InterruptedException
    {
        int zeroedEncoders = leftDrive1.getCurrentPosition();
        double loopSpeed;
        double previousTime = 0.0;
        double currentTime;
        driveTimer.reset();
        integratingGyro.resetZAxisIntegrator();
        int backupTime = 200;
        double xDegreesTurned = 0.0;
        double error;
        double gain = 0.05; // gain to multiply error by

        while (absoluteValue(leftDrive1.getCurrentPosition()-zeroedEncoders) < clicks
                && driveTimer.time() < timeout) {
            waitOneFullHardwareCycle();
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

    // Turn clockwise with gyro
    void turnClockwise (double motorPower, int degrees) throws InterruptedException {
        while (headingShift(integratingGyro.getHeading()) < degrees) {
            waitOneFullHardwareCycle();
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
    void turnCounterClockwise (double motorPower, int degrees) throws InterruptedException {
        while (headingShift(integratingGyro.getHeading()) > -degrees) {
            waitOneFullHardwareCycle();
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

    int absoluteValue(int value) {
        if(value >= 0)
            return value;
        else
            return -value;
    }
}