/*
Notes:

Adding anti-stick-on-debris for the other direction would be cool (and useful because I do drive
that direction when going back into the beacon repair zone), but if I don't add it then I don't
need to worry about making it a parameter. Test if the robot has trouble with debris when driving
that way. Also keep in mind that when driving to the beacon repair zone it has already cleared the
path (when driving away form it), and that if it sticks on debris when starting to go up the
mountain then it won't be able to sense that regardless (because it tilts back when climbing the
mountain anyway)

Every configuration should be tested

Make wait clever (time options and adjust allowed wait based on that)

Add an option for driving into opposing alliance's area (but make sure that it doesn't happen
before the 10 second mark)

Check autonomous time (right now it's possible that I could get a penalty for driving into that
corner and the opposing alliance's side before 10 seconds)

While waiting in that corner run gyro correction so that if someone bumps the robot we correct

If gyro wasn't calibrated automatically calibrate it at the beginning of autonomous

Make sure that parking on opposing ramp is legal
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class autonomous10 extends LinearOpMode {
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

    // Button booleans
    boolean right = false, left = false, lastRight = false, lastLeft = false;

    int variable = 0; // generic variable for method to adjust

    int color = 0; // 0 = red, 1 = blue
    int startPos = 0; // 0 = close, 1 = far
    int whichAuto = 0; // 0 = full, 1 = mountain
    int waitTime = 0; // Time to wait (in seconds)
    int endPos = 0; // 0 = beacon repair zone (leave then return), 1 = corner, 2 = beacon repair zone (never leave)

    public void runOpMode() throws InterruptedException {
        getReference(); // Get reference to all hardware

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        // Run with no encoders (I think this is default, need to check)
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

        autonomousMenu(); // Run full autonomous menu

        waitForStart();
        autoTimer.reset();
        while (autoTimer.time() < waitTime) // Wait 0-10 seconds before starting (determined by menu)
            sleep(50);

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);

        // Reverse brush to clear debris
        leftBrushMotor.setPower(-1.0);
        rightBrushMotor.setPower(-1.0);
            if (startPos == 0) { // Near
                if (whichAuto == 0) { // Near full
                    // Drive away from wall
                    gyroCorrection(-0.35, 100, 1.0);
                    gyroCorrection(-1.0, 2900, 4.0);

                    // Turn towards shelter
                    if (color == 0) // If red
                        turnCounterClockwise(1.0, 46, 3.0);
                    else // If blue
                        turnClockwise(1.0, 46, 3.0);

                    // Drive to shelter
                    gyroCorrection(-0.35, 100, 1.0);
                    gyroCorrection(-1.0, 7900, 7.5);

                    // Stop brush
                    leftBrushMotor.setPower(0.0);
                    rightBrushMotor.setPower(0.0);

                    // Place climbers in shelter
                    if (color == 0) {
                        rightAutoServo.setPosition(rightAutoOut);
                        sleep(2000); // Wait for servo to extend

                        // Retract servo
                        rightAutoServo.setPosition(rightAutoIn);
                    } else {
                        leftAutoServo.setPosition(leftAutoOut);
                        sleep(2000); // Wait for servo to extend

                        // Retract servo
                        leftAutoServo.setPosition(leftAutoIn);
                    }
                    sleep(1000); // Wait for servos to retract (play with this time)

                    if (endPos != 2) { // If moving after scoring
                        // Back away from shelter (so that robot doesn't catch when turning)
                        gyroCorrection(0.3, 100, 1.0);
                        gyroCorrection(1.0, 1900, 1.5);

                        if (color == 0) // If red
                            turnClockwise(1.0, 46, 3.0);
                        else // if blue
                            turnCounterClockwise(1.0, 46, 3.0);
                    }
                } else { // Near mountain
                    // Drive away from wall
                    gyroCorrection(-1.0, 2500, 4.0); // might start too fast and have trouble

                    // Turn to mountain
                    if(color == 0) // if red
                        turnCounterClockwise(1.0, 90, 3.0);
                    else // if blue
                        turnClockwise(1.0, 90, 3.0);

                    // Park on mountain
                    gyroCorrection(-1.0, 1800, 2.0);

                    // Stop brush
                    leftBrushMotor.setPower(0.0);
                    rightBrushMotor.setPower(0.0);
                }
            } else { // Far
                if(whichAuto == 0) { // Far full
                    // Drive away from wall
                    gyroCorrection(-0.35, 800, 1.5);

                    // Turn towards shelter
                    if(color == 0) // if red
                        turnCounterClockwise(0.75, 50, 3.0);
                    else // if blue
                        turnClockwise(0.75, 50, 3.0);

                    // Drive to shelter
                    gyroCorrection(-0.35, 13000, 8.5);

                    // Stop brush
                    leftBrushMotor.setPower(0.0);
                    rightBrushMotor.setPower(0.0);

                    // Place climbers in shelter
                    if (color == 0) {
                        rightAutoServo.setPosition(rightAutoOut);
                        sleep(2000); // Wait for servo to extend

                        // Retract servo
                        rightAutoServo.setPosition(rightAutoIn);
                    } else {
                        leftAutoServo.setPosition(leftAutoOut);
                        sleep(2000); // Wait for servo to extend

                        // Retract servo
                        leftAutoServo.setPosition(leftAutoIn);
                    }
                    sleep(1000); // Wait for servos to retract (play with this time)

                    if (endPos != 2) {
                        // Back away from shelter (so that robot doesn't catch when turning)
                        gyroCorrection(0.3, 1400, 1.5);

                        if (color == 0) // if red
                            turnClockwise(.65, 50, 3.0);
                        else
                            turnCounterClockwise(.65, 50, 3.0);
                    }
                } else { // Far mountain
                    // Drive away from wall
                    gyroCorrection(-1.0, 800, 4.0);

                    // Turn towards mountain
                    if(color == 0) // If red
                        turnCounterClockwise(1.0, 75, 3.0);
                    else // If blue
                        turnClockwise(1.0, 90, 3.0);

                    // Drive onto mountain
                    gyroCorrection(-1.0, 7000, 8.0);
                }
            }

        if(whichAuto == 0 && endPos != 2) { // if running full autonomous and moving after scoring
            leftBrushMotor.setPower(-1.0);
            rightBrushMotor.setPower(-1.0);
            gyroCorrection(-1.0, 6000, 4.0);
            leftBrushMotor.setPower(0.0);
            rightBrushMotor.setPower(0.0);

            if (endPos == 0) {
                while (autoTimer.time() < 26)
                    sleep(50);
                gyroCorrection(0.5, 4000, 4.0);
            }
        }
    }


    int menuItem(int maxNumber) throws InterruptedException {
        telemetry.addData("When finished press", "\"b\"");
        right = gamepad1.dpad_right;
        left = gamepad1.dpad_left;
        if (right && !lastRight) {
            variable++;
            if(variable >= maxNumber)
                variable = 0;
        } else if (left && !lastLeft) {
            variable--;
            if(variable < 0)
                variable = maxNumber-1;
        }
        lastRight = right;
        lastLeft = left;
        return variable;
    }

    // Clear screen, reset variable, wait so button press doesn't register twice
    // This method should be called after every while loop with a new menu item
    void reset() throws InterruptedException {
        telemetry.clearData();
        sleep(500); // So that button press isn't read in next menu option
        variable = 0;
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
            telemetry.addData("Clicks: ", absoluteValue(leftDrive1.getCurrentPosition()-zeroedEncoders)); // Distance driven
            waitOneFullHardwareCycle(); // Without this line auto won't stop via the driver station
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
    void turnClockwise (double motorPower, int degrees, double timeout) throws InterruptedException {
        driveTimer.reset();
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
    void turnCounterClockwise (double motorPower, int degrees, double timeout) throws InterruptedException {
        driveTimer.reset();
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

    int absoluteValue(int value) {
        if(value >= 0)
            return value;
        return -value;
    }

    void autonomousMenu() throws InterruptedException {

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
