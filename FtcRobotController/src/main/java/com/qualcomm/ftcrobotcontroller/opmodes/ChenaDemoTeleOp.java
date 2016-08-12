/*
Tele-op program to be used at Worlds
Use one gamepad to control drivetrain, collection, lift, and twelve servos
Also use autonomous operations to simplify driving
*/

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class ChenaDemoTeleOp extends OpMode
{
    // Declaring motors and servos
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, liftMotor1, liftMotor2,
            leftBrushMotor, rightBrushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo,
            rightFenderServo, leftFenderServo;
    TouchSensor touchSensor;
    ColorSensor colorSensor;

    int task = 1; // Current task (used in rampPos method)

    float xValue; // X Value of joystick
    float yValue; // Y value of joystick
    double zeroEncoders; // Hold encoder value from program start
    float liftDeadband = 0.1F; // Deadband for trigger which moves lift
    // Servo positions
    double rightLiftUp = 0.0;
    double rightLiftDown = 0.3;
    double leftLiftUp = 1.0;
    double leftLiftDown = 0.7;
    double leftAutoIn = 0.06;
    double leftAutoOut = 1.0;
    double rightAutoIn = 0.95;
    double rightAutoOut = 0.0;
    double leftDebrisOut = 0.51;
    double leftDebrisIn = 0.424;
    double rightDebrisOut = 0.914;
    double rightDebrisIn = 1.0;
    double releaseLift = 1.0;
    double brakeLift = 0.824;
    double rightZipIn = 0.8;
    double rightZipOut = 0.1;
    double leftZipIn = 0.2;
    double leftZipOut = 0.85;
    double toggleClose = .45;
    double toggleOpen = .55;
    double leftFenderDown = 0.5;
    double rightFenderDown = 0.68;
    double leftFenderUp = 1.0;
    double rightFenderUp = 0.1;
    // Booleans to decide which action to perform
    boolean leftDebrisScore = false;
    boolean rightDebrisScore = false;
    boolean negateMotor = false;
    boolean lockBrake = false;
    boolean brushForward = false;
    boolean brushReverse = false;
    boolean zipOut = false;
    boolean autoOut = false;
    boolean fendersDown = true;
    // Previous value from gamepad buttons
    boolean lastX = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastLeftDpad = false;
    boolean lastRightDpad = false;
    boolean lastGuide = false;
    boolean lastLeftTrigger = false;
    boolean lastLeftBumper = false;
    boolean lastLeftJoystick = false;
    boolean lastBack = false;
    // Hold value from gamepad buttons (for less frequent retrieval)
    boolean xButton = false;
    boolean guide = false;
    boolean leftDpad = false;
    boolean rightDpad = false;
    boolean leftTrigger = false;
    boolean leftBumper = false;
    boolean aButton = false;
    boolean bButton = false;
    boolean leftJoystick = false;
    float rightTrigger = 0.0F;

    public void init() // Code to run during initialization
    {
        // Get references to hardware
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        leftBrushMotor = hardwareMap.dcMotor.get("leftBrushMotor");
        rightBrushMotor = hardwareMap.dcMotor.get("rightBrushMotor");
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
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        // Reverse right drive motors and left brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        zeroEncoders = liftMotor2.getCurrentPosition(); // Set zero to where encoders started

        // Also use encoders on one brush motor (for accurate low speeds)
        rightBrushMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

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
        rightFenderServo.setPosition(rightFenderDown);
        leftFenderServo.setPosition(leftFenderDown);

        colorSensor.enableLed(true); // Turn on color sensor (to show current front)
    }

    @Override
    public void loop () // Main tele-op code
    {
        telemetry.addData("Lift position: ", getPercentage(liftMotor2.getCurrentPosition()
                - zeroEncoders)+"%"); // Display lift position

        xButton = gamepad1.x; // For less frequent retrieval
        negateMotor = updateButton(xButton, lastX, negateMotor); // Was button pressed?
        lastX = xButton; // Update last button value

        // Retrieve value from joystick y axis and negate it (to reverse fronts) if necessary
        // Also update color sensor to show current front
        if (negateMotor) {
            yValue = -gamepad1.left_stick_y;
            colorSensor.enableLed(false);
        } else {
            yValue = gamepad1.left_stick_y;
            colorSensor.enableLed(true);
        }

        xValue = gamepad1.left_stick_x; // Get x value of left joystick
        // Calculate motor power, clip, and update motors
        float leftPower = Range.clip(yValue + xValue, -0.6F, 0.6F);
        float rightPower = Range.clip(yValue - xValue, -0.6F, 0.6F);
        // Only set motor power here if debris is not being scored. If it is being scored, then it
        // is set in that code (and adjusted slightly to make lining up on the goal easier)
            leftDrive1.setPower(leftPower);
            leftDrive2.setPower(leftPower);
            rightDrive1.setPower(rightPower);
            rightDrive2.setPower(rightPower);

        rightTrigger = gamepad1.right_trigger; // For less frequent retrieval
        if (gamepad1.right_bumper && (Integer.parseInt(getPercentage(liftMotor2.getCurrentPosition()
                - zeroEncoders)) <= 30)) // Raise lift while pressed
        {
            liftMotor1.setPower(1.0);
            liftMotor2.setPower(1.0);
        } else if (rightTrigger >= liftDeadband && (Integer.parseInt(getPercentage(liftMotor2.getCurrentPosition()
                - zeroEncoders)) > 1) && !leftDebrisScore && !rightDebrisScore) // Lower lift while pressed more than deadband
        {
            // Set motors to trigger values (0.1-1.0) for precise control
            liftMotor1.setPower(-rightTrigger);
            liftMotor2.setPower(-rightTrigger);
        } else { // If lift is not being moved in other code, set power to zero
            liftMotor1.setPower(0.0);
            liftMotor2.setPower(0.0);
        }

        /*
        Brush is controlled by two buttons. When one is pressed it goes forward. If the same button
        is pressed again then it stops. When the other button is pressed it goes backwards, and if
        the same button is pressed again it stops. If the brush is going forwards and the reverse
        button is pressed, then it will go backwards.
        Also, whenever the brush is not being told to go forwards or backwards, it rotates forward
        until it presses a touch sensor. This is so that it is guaranteed to be in a certain
        position when not in use (if it is in a bad position it can interfere with climbing
        the mountain).
        NOTE: Button press method is not used in this code because multiple variables are updated
        if a button is pressed
        */
        leftBumper = gamepad1.left_bumper; // For less frequent retrieval
        // Trigger returns from 0-1, only register press if it is pressed more than halfway
        leftTrigger = gamepad1.left_trigger >= 0.5;
        if(leftBumper && !lastLeftBumper)
        {
            brushForward = !brushForward;
            brushReverse = false;
        }
        else if (leftTrigger && !lastLeftTrigger)
        {
            brushReverse = !brushReverse;
            brushForward = false;
        }
        lastLeftTrigger = leftTrigger; // Update last value
        lastLeftBumper = leftBumper; // Update last value

        // Update brush powers
        if (brushForward) {
            leftBrushMotor.setPower(1.0);
            rightBrushMotor.setPower(1.0);
        } else if (brushReverse) {
            leftBrushMotor.setPower(-1.0);
            rightBrushMotor.setPower(-1.0);
        } else {
            // If the touch sensor is pressed then stop, otherwise move forward slowly
            if (touchSensor.isPressed()) {
                leftBrushMotor.setPower(0.0);
                rightBrushMotor.setPower(0.0);
            } else {
                // Powers are different because one motor runs with an encoder, other just follows
                leftBrushMotor.setPower(0.05); // Following at slower speed
                rightBrushMotor.setPower(0.08); // Motor with encoder
            }
        }

        // For less frequent retrieval
        if ((Integer.parseInt(getPercentage(liftMotor2.getCurrentPosition() - zeroEncoders)) > 25))
            leftDpad = gamepad1.dpad_left;
        else
            leftDpad = false;
        if ((Integer.parseInt(getPercentage(liftMotor2.getCurrentPosition() - zeroEncoders)) > 25))
            rightDpad = gamepad1.dpad_right;
        else
            rightDpad = false;
        leftDebrisScore = updateButton(leftDpad, lastLeftDpad, leftDebrisScore); // Move servo?
        lastLeftDpad = leftDpad; // Update last value
        rightDebrisScore = updateButton(rightDpad, lastRightDpad, rightDebrisScore); // Move servo?
        lastRightDpad = rightDpad; // Update last value
        // Update debris servos as necessary
        if (leftDebrisScore)
            leftDebrisServo.setPosition(leftDebrisOut);
        else
            leftDebrisServo.setPosition(leftDebrisIn);
        if (rightDebrisScore)
            rightDebrisServo.setPosition(rightDebrisOut);
        else
            rightDebrisServo.setPosition(rightDebrisIn);

        aButton = gamepad1.a; // For less frequent retrieval
        zipOut = updateButton(aButton, lastA, zipOut); // Move servo?
        lastA = aButton; // Update last button value
        // Update zipline servos
        if (zipOut) {
            rightZipServo.setPosition(rightZipOut);
            leftZipServo.setPosition(leftZipOut);
        } else {
            rightZipServo.setPosition(rightZipIn);
            leftZipServo.setPosition(leftZipIn);
        }

        bButton = gamepad1.b; // For less frequent retrieval
        autoOut = updateButton(bButton, lastB, autoOut); // Move servo?
        lastB = bButton; // Update last value
        // Update autonomous servos
        if(autoOut) {
            rightAutoServo.setPosition(rightAutoOut);
            leftAutoServo.setPosition(leftAutoOut);
        } else {
            rightAutoServo.setPosition(rightAutoIn);
            leftAutoServo.setPosition(leftAutoIn);
        }

        leftJoystick = gamepad1.start; // For less frequent retrieval
        fendersDown = updateButton(leftJoystick, lastLeftJoystick, fendersDown); // Move servo?
        lastLeftJoystick = leftJoystick; // Update last value

        // Update fender servos
        if(fendersDown) {
            leftFenderServo.setPosition(leftFenderDown);
            rightFenderServo.setPosition(rightFenderDown);
        } else {
            leftFenderServo.setPosition(leftFenderUp);
            rightFenderServo.setPosition(rightFenderUp);
        }

        if(gamepad1.right_stick_button) // If joystick pressed, release debris
            toggleServo.setPosition(toggleOpen);
        else // Otherwise hold debris in
            toggleServo.setPosition(toggleClose);
    } // loop method

    String getPercentage(double currentPos) { // Get percentage that lift is extended
        int percentage = (int)(currentPos/-11037*100); // Calculate percentage lift is extended
        return Integer.toString(percentage); // Return as a string
    }

    // If button is pressed and was not pressed last iteration, change value. This is because
    // the button can be pressed and held briefly. If the program registered both presses then
    // value would change twice and cancel out.
    boolean updateButton(boolean button, boolean lastButton, boolean updateValue) {
        if(button && !lastButton)
            updateValue = !updateValue;
        return updateValue;
    }
} // TeleOpRed2