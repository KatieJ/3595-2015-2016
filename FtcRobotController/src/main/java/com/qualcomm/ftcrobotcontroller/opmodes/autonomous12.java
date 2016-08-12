/*
Autonomous program to be used at Worlds. Menu allows many different options. There are two base
programs -- one scores climbers and parks in the beacon repair zone, the other parks on the
mountain. Each of these programs is then adjusted based on what color we are, how long to wait
before moving, and what our alliance partner will be doing.
All options are selected in an autonomous menu, which uses the gamepad to select specifications
for each match.
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class autonomous12 extends LinearOpMode {
    // Declaring motors, servos, and sensors
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, leftBrushMotor, rightBrushMotor,
            liftMotor1, liftMotor2;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo,
            leftFenderServo, rightFenderServo;
    GyroSensor integratingGyro;

    public ElapsedTime driveTimer = new ElapsedTime();
    public ElapsedTime autoTimer = new ElapsedTime();

    int i = 0; // iterations of while loop
    double leftMotorPower = 0.0, rightMotorPower = 0.0, offset = 0.0, gyroReadings = 0.0;

    // Positions for different servos
    double rightLiftUp = 0.0;
    double leftLiftUp = 1.0;
    double leftAutoIn = 0.06;
    double leftAutoOut = 1.0;
    double rightAutoIn = 0.95;
    double rightAutoOut = 0.0;
    double leftDebrisIn = 0.424;
    double rightDebrisIn = 1.0;
    double releaseLift = 1.0;
    double rightZipIn = 1.0;
    double leftZipIn = 0.0;
    double toggleClose = 0.45;
    double leftFenderDown = 0.5;
    double rightFenderDown = 0.68;
    double leftFenderUp = 1.0;
    double rightFenderUp = 0.1;

    // Button booleans
    boolean right = false, left = false, lastRight = false, lastLeft = false;

    int menuOption = 0; // generic variable for method to adjust

    int color = 0; // 0 = red, 1 = blue
    int startPos = 0; // 0 = close, 1 = far
    int whichAuto = 0; // 0 = full, 1 = mountain
    int waitTime = 0; // Time to wait (in seconds)
    int endPos = 0; // 0 = beacon repair zone (leave then return), 1 = corner,
    // 2 = beacon repair zone (never leave)

    public void runOpMode() throws InterruptedException {
        getReference(); // Get reference to all hardware

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        // Run with no encoders
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
        leftFenderServo.setPosition(leftFenderDown);
        rightFenderServo.setPosition(rightFenderDown);

        // So that robot is maneuverable during set-up
        leftDrive1.setPowerFloat();
        leftDrive2.setPowerFloat();
        rightDrive1.setPowerFloat();
        rightDrive2.setPowerFloat();
        liftMotor1.setPowerFloat();
        liftMotor2.setPowerFloat();

        autonomousMenu(); // Run autonomous menu

        waitForStart();
        autoTimer.reset();
        while (autoTimer.time() < waitTime) // Wait 0-10 seconds before start (determined by menu)
            sleep(50);

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);
        if (startPos == 0) { // Near
            if (whichAuto == 0) { // Near full
                // Drive away from wall
                gyroCorrection(-1.0, 3000, 4.0, true);

                // Turn towards shelter
                if (color == 0) // If red
                    turnCounterClockwise(1.0, 47, 3.0);
                else // If blue
                    turnClockwise(1.0, 46, 3.0);

                // Drive to shelter
                gyroCorrection(-1.0, 7000, 7.5, true);
                // Drive final distance slowly, so robot doesn't slam into wall
                gyroCorrection(-0.3, 1000, 1.5, true);

                // Stop brush
                leftBrushMotor.setPower(0.0);
                rightBrushMotor.setPower(0.0);

                // Place climbers in shelter
                if (color == 0) { // If red
                    rightAutoServo.setPosition(rightAutoOut);
                    sleep(2000); // Wait for servo to extend
                    rightAutoServo.setPosition(rightAutoIn);
                } else { // If blue
                    leftAutoServo.setPosition(leftAutoOut);
                    sleep(2000); // Wait for servo to extend
                    leftAutoServo.setPosition(leftAutoIn);
                }
                sleep(1000); // Wait for servo

                if (endPos != 2) { // If moving after scoring
                    // Back away from shelter (so that robot doesn't catch when turning)
                    gyroCorrection(1.0, 2000, 1.5, true);

                    if (color == 0) // If red
                        turnClockwise(1.0, 47, 3.0);
                    else // If blue
                        turnCounterClockwise(1.0, 46, 3.0);
                }
            } else { // Near mountain
                // Drive away from wall
                gyroCorrection(-1.0, 2500, 4.0, true);

                // Turn to mountain
                if(color == 0) // If red
                    turnCounterClockwise(1.0, 90, 3.0);
                else // If blue
                    turnClockwise(1.0, 90, 3.0);

                // Drive forward, raise fenders, then finish driving forward
                gyroCorrection(-1.0, 1300, 2.0, true);
                leftFenderServo.setPosition(leftFenderUp);
                rightFenderServo.setPosition(rightFenderUp);
                gyroCorrection(-1.0, 800, 2.0, false);

                // Stop brush
                leftBrushMotor.setPower(0.0);
                rightBrushMotor.setPower(0.0);
            }
        } else { // Far
            if(whichAuto == 0) { // Far full
                // Drive away from wall
                gyroCorrection(-1.0, 800, 1.5, true);

                // Turn towards shelter
                if(color == 0) // If red
                    turnCounterClockwise(0.75, 55, 3.0);
                else // If blue
                    turnClockwise(0.75, 55, 3.0);

                // Drive to shelter
                gyroCorrection(-1.0, 12000, 7.5, true);
                gyroCorrection(-0.3, 1000, 1.5, true);

                // Stop brush
                leftBrushMotor.setPower(0.0);
                rightBrushMotor.setPower(0.0);

                // Place climbers in shelter
                if (color == 0) { // If red
                    rightAutoServo.setPosition(rightAutoOut);
                    sleep(2000); // Wait for servo to extend
                    rightAutoServo.setPosition(rightAutoIn);
                } else { // If blue
                    leftAutoServo.setPosition(leftAutoOut);
                    sleep(2000); // Wait for servo to extend
                    leftAutoServo.setPosition(leftAutoIn);
                }
                sleep(1000); // Wait for servo to retract

                if (endPos != 2) { // If moving after scoring
                    // Back away from shelter (so that robot doesn't catch when turning)
                    gyroCorrection(1.0, 1400, 1.5, true);

                    if (color == 0) // If red
                        turnClockwise(0.75, 55, 3.0);
                    else // If blue
                        turnCounterClockwise(0.75, 55, 3.0);
                }
            } else { // Far mountain
                // Drive away from wall
                gyroCorrection(-1.0, 800, 4.0, true);

                // Turn towards mountain
                if(color == 0) // If red
                    turnCounterClockwise(1.0, 74, 3.0);
                else // If blue
                    turnClockwise(1.0, 74, 3.0);

                // Drive forward, raise fenders, then finish driving forward
                gyroCorrection(-1.0, 5500, 7.0, true);
                leftFenderServo.setPosition(leftFenderUp);
                rightFenderServo.setPosition(rightFenderUp);
                gyroCorrection(-1.0, 2000, 3.0, false);

            }
        }

        if(whichAuto == 0 && endPos != 2) { // If running full autonomous and moving after scoring
            leftBrushMotor.setPower(-1.0);
            rightBrushMotor.setPower(-1.0);
            gyroCorrection(-1.0, 6000, 4.0, true); // Drive to corner
            leftBrushMotor.setPower(0.0);
            rightBrushMotor.setPower(0.0);

            // If driving back to beacon repair zone, wait for final 4 seconds of autonomous
            if (endPos == 0) {
                while (autoTimer.time() < 26)
                    sleep(50);
                gyroCorrection(0.5, 4000, 4.0, true); // Back up into beacon repair zone
            }
        }
    }

    // Generic method for changing an option on the menu
    int menuItem(int maxNumber) throws InterruptedException {
        telemetry.addData("When finished press", "\"b\"");
        right = gamepad1.dpad_right;
        left = gamepad1.dpad_left;
        if (right && !lastRight) { // If button is pressed and was not last time, increment value
            menuOption++;
            if(menuOption >= maxNumber) // If too large, roll over to 0
                menuOption = 0;
        } else if (left && !lastLeft) {  // If button is pressed and was not last time, decrement value
            menuOption--; // If too small, roll over to max number
            if(menuOption < 0)
                menuOption = maxNumber-1;
        }
        // Update last button values
        lastRight = right;
        lastLeft = left;
        return menuOption;
    }

    // Clear screen, reset variable, wait so button press doesn't register twice
    // This method should be called after every while loop with a new menu item
    void reset() throws InterruptedException {
        telemetry.clearData();
        sleep(500);
        menuOption = 0;
    }

    // Calculate error and adjust motor powers accordingly
    // Final parameter ("backup") is so that anti-tilt can be turned off. This is so that this
    // method can be used to park on the mountain (which tilts the robot backwards).
    void gyroCorrection(double motorPower, int clicks, double timeout, boolean backup)
            throws InterruptedException
    {
        int zeroedEncoders = leftDrive1.getCurrentPosition();
        double loopSpeed;
        double previousTime = 0.0;
        double currentTime;
        driveTimer.reset();
        integratingGyro.resetZAxisIntegrator();
        double xDegreesTurned = 0.0;
        double error;
        double gain = 0.05; // gain to multiply error by

        // While robot has not driven far enough AND timeout has not been reached
        while (absoluteValue(leftDrive1.getCurrentPosition()-zeroedEncoders) < clicks
                && driveTimer.time() < timeout) {
            waitOneFullHardwareCycle(); // Without this line auto won't stop via the driver station
            currentTime = driveTimer.time();
            loopSpeed = currentTime - previousTime;
            previousTime = currentTime;

            // Calculate distance tilted back
            xDegreesTurned += (integratingGyro.rawX() - offset) * loopSpeed;

            error = headingShift(integratingGyro.getHeading());

            if (xDegreesTurned > 250 && backup) { // If robot is on debris, back up
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
            if (xDegreesTurned > 250 && backup)
                sleep(200);
        }
        // Brake drive motors when method is done
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }

    // Turn clockwise with gyro
    void turnClockwise (double motorPower, int degrees, double timeout)throws
            InterruptedException {
        driveTimer.reset();
        // While robot has not turned far enough (and has not reached timeout), keep turning
        while (headingShift(integratingGyro.getHeading()) < degrees && driveTimer.time() < timeout) {
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
    void turnCounterClockwise (double motorPower, int degrees, double timeout)
            throws InterruptedException {
        driveTimer.reset();
        // While robot has not turned far enough (and has not reached timeout), keep turning
        while (headingShift(integratingGyro.getHeading()) > -degrees && driveTimer.time() < timeout) {
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

    // Heading returns a number from 0 to 360, adjust so that a number -180 to 180 is returned
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
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
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
        leftFenderServo = hardwareMap.servo.get("leftFenderServo");
        rightFenderServo = hardwareMap.servo.get("rightFenderServo");
        // Get reference to gyro
        integratingGyro = hardwareMap.gyroSensor.get("integratingGyro");
    }

    // Get absolute value of number
    int absoluteValue(int value) {
        if(value >= 0)
            return value;
        return -value;
    }

    void autonomousMenu() throws InterruptedException {
        // Menu to select different autonomous options
        while(!gamepad1.b) {
            waitOneFullHardwareCycle();
            color = menuItem(2);
            if (color == 0)
                telemetry.addData("Color (change with arrows)", " Red");
            else
                telemetry.addData("Color (change with arrows)", " Blue");
        }
        reset();

        while(!gamepad1.b) {
            waitOneFullHardwareCycle();
            startPos = menuItem(2);
            if (startPos == 0)
                telemetry.addData("Start position (change with arrows)", " Close");
            else
                telemetry.addData("Start position (change with arrows)", " Far");
        }
        reset();

        while(!gamepad1.b) {
            waitOneFullHardwareCycle();
            whichAuto = menuItem(2);
            if(whichAuto == 0)
                telemetry.addData("Which autonomous? (change with arrows)", "Full");
            else
                telemetry.addData("Which autonomous? (change with arrows)", "Just mountain");
        }
        reset();

        while(!gamepad1.b) {
            waitOneFullHardwareCycle();
            waitTime = menuItem(11);
            telemetry.addData("Time to wait (change with arrows)", Integer.toString(waitTime));
        }
        reset();

        if(whichAuto == 0) {
            while (!gamepad1.b) {
                waitOneFullHardwareCycle();
                endPos = menuItem(3);
                if (endPos == 0)
                    telemetry.addData("End position (change with arrows)", "Beacon repair zone (leave then return");
                else if (endPos == 1)
                    telemetry.addData("End position (change with arrows)", "Corner/opposing side");
                else
                    telemetry.addData("End position (change with arrows)", "Beacon repair zone (never leave");
            }
            reset();
        }

        while(!gamepad1.b) {
            waitOneFullHardwareCycle();
            telemetry.addData("Press \"b\" to calibrate gyro", "");
        }
        calibrateGyro();
        telemetry.clearData();
        telemetry.addData("Gyro: ", "calibrated");
        sleep(1000); // Wait so that previous message can be seen
        telemetry.clearData();

        displayChoices();
    }

    void displayChoices() {
        // Display all choices from menu
        if (color == 0)
            telemetry.addData("Selected color", "Red");
        else
            telemetry.addData("Selected color", "Blue");

        if (startPos == 0)
            telemetry.addData("Selected start position", "Close");
        else
            telemetry.addData("Selected start position", "Far");

        if(whichAuto == 0)
            telemetry.addData("Selected autonomous", "Full");
        else
            telemetry.addData("Selected autonomous", "Just mountain");

        // Int cast to a string because telemetry cannot display an int
        telemetry.addData("Selected wait time", Integer.toString(waitTime));

        if (whichAuto == 1) {
            telemetry.addData("Selected end position", "N/A");
        }
        else if (endPos == 0)
            telemetry.addData("Selected end position", "Beacon repair zone");
        else
            telemetry.addData("Selected end position", "Corner/opposing side");
    }
}
