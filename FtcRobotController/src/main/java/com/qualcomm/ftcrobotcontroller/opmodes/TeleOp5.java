package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

// General tele-op program for robot (this iteration started on March 5th)
// Allow hanging, control of ten different servos, and debris scoring

public class TeleOp5 extends OpMode
{
    // Declaring motors and servos
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, liftMotor1, liftMotor2, leftBrushMotor, rightBrushMotor;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo;
    TouchSensor touchSensor;

    float xValue; // X Value of joystick
    float yValue; // Y value of joystick

    // Positions for different servos
    double rightLiftUp = 0.0;
    double rightLiftDown = 0.3;
    double leftLiftUp = 1.0;
    double leftLiftDown = 0.7;
    double leftAutoIn = 0.0;
    double leftAutoOut = 1.0;
    double rightAutoIn = 0.95;
    double rightAutoOut = 0.0;
    double leftDebrisOut = 0.510;
    double leftDebrisIn = 0.424;
    double rightDebrisOut = 0.914;
    double rightDebrisIn = 1.0;
    double releaseLift = 1.0;
    double brakeLift = 0.824;
    double rightZipIn = 1.0;
    double rightZipOut = 0.1;
    double leftZipIn = 0.0;
    double leftZipOut = 0.85;
    double toggleClose = 0.824;
    double toggleOpen = 0.902;

    // Booleans to decide which action to perform
    boolean leftDebrisScore = false;
    boolean rightDebrisScore = false;
    boolean negateMotor = true;
    boolean lockBrake = false;
    boolean brushForward = false;
    boolean brushReverse = false;
    boolean zipOut = false;
    boolean autoOut = false;

    // Previous value from gamepad button
    boolean lastX = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastLeftDpad = false;
    boolean lastRightDpad = false;
    boolean lastY = false;
    boolean lastLeftTrigger = false;
    boolean lastLeftBumper = false;
    boolean lastJoystick = false;

    // Holds value from gamepad button (for less frequent retrieval)
    boolean gamepadX = false;
    boolean gamepadY = false;
    boolean gamepadLeftDpad = false;
    boolean gamepadRightDpad = false;
    boolean leftTrigger = false;
    boolean leftBumper = false;
    boolean gamepadA = false;
    boolean gamepadB = false;
    boolean joystick = false;

    public void init() // Code to run during initialization
    {
        // Get references to motors
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        leftBrushMotor = hardwareMap.dcMotor.get("leftBrushMotor");
        rightBrushMotor = hardwareMap.dcMotor.get("rightBrushMotor");

        // Reverse right drive motors and brush motor
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBrushMotor.setDirection(DcMotor.Direction.REVERSE);

        rightBrushMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

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

        touchSensor = hardwareMap.touchSensor.get("touchSensor");

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
    }

    @Override
    public void loop () // Main tele-op code
    {
        // When button x is pressed the readings from y axis on the joystick are negated, thus
        // reversing the robot's front
        gamepadX = gamepad1.x; // update variable value

        // If button is pressed (and was not last time), then reverse fronts
        // This is so that if the button is pressed and held for a moment, the fronts don't reverse
        // twice, making the press ineffective
        if (gamepadX & !lastX)
            negateMotor = !negateMotor;
        lastX = gamepadX; // Update holder for previous value

        // Retrieve value from joystick y axis and negate it if necessary
        if (negateMotor)
            yValue = -gamepad1.left_stick_y;
        else
            yValue = gamepad1.left_stick_y;

        // Get x value of left joystick and calculate motor power
        xValue = gamepad1.left_stick_x;
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        // clip motor powers to within range
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // set motors to determined power
        leftDrive1.setPower(leftPower);
        leftDrive2.setPower(leftPower);
        rightDrive1.setPower(rightPower);
        rightDrive2.setPower(rightPower);

        if (gamepad1.right_bumper) // Raise lift while right bumper pressed
        {
            liftMotor1.setPower(1.0);
            liftMotor2.setPower(1.0);
        }
        else if (gamepad1.right_trigger >= 0.5) // Lower lift while right trigger pressed > halfway
        {
            liftMotor1.setPower(-1.0);
            liftMotor2.setPower(-1.0);
        }
        else // Otherwise brake motors
        {
            liftMotor1.setPower(0.0);
            liftMotor2.setPower(0.0);
        }

        // Code for controlling brush: If one button is pressed, toggle between forward and stopped
        // If other button is pressed, toggle between reversed and stopped
        leftBumper = gamepad1.left_bumper;
        leftTrigger = gamepad1.left_trigger >= 0.5;
        if(leftBumper & !lastLeftBumper)
        {
            brushForward = !brushForward;
            brushReverse = false;
        }
        else if (leftTrigger & !lastLeftTrigger)
        {
            brushReverse = !brushReverse;
            brushForward = false;
        }
        lastLeftTrigger = leftTrigger; // Update last value
        lastLeftBumper = leftBumper; // Update last value

        // Update brush powers
        if (brushForward) {
            rightBrushMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            leftBrushMotor.setPower(1.0);
            rightBrushMotor.setPower(1.0);
        } else if (brushReverse) {
            rightBrushMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            leftBrushMotor.setPower(-1.0);
            rightBrushMotor.setPower(-1.0);
        } else {
            rightBrushMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            if (touchSensor.isPressed()) {
                leftBrushMotor.setPower(0.0);
                rightBrushMotor.setPower(0.0);
            } else {
                leftBrushMotor.setPower(0.08);
                rightBrushMotor.setPower(0.1);
            }
        }

        if (gamepad1.dpad_up) // Rotate lift up when dpad up pressed
        {
            rightLiftServo.setPosition(rightLiftUp);
            leftLiftServo.setPosition(leftLiftUp);
        }
        else if (gamepad1.dpad_down) // Rotate lift down when dpad down pressed
        {
            rightLiftServo.setPosition(rightLiftDown);
            leftLiftServo.setPosition(leftLiftDown);
        }

        gamepadLeftDpad = gamepad1.dpad_left;
        gamepadRightDpad = gamepad1.dpad_right;

        // If button is pressed (and was not last time), set servo to be updated
        if (gamepadLeftDpad & !lastLeftDpad)
            leftDebrisScore = !leftDebrisScore;
        lastLeftDpad = gamepadLeftDpad; // Update last button value

        // If button is pressed (and was not last time), set servo to be updated
        if (gamepadRightDpad & !lastRightDpad)
            rightDebrisScore = !rightDebrisScore;
        lastRightDpad = gamepadRightDpad; // Update last button value

        // Update autonomous servos as necessary
        if (leftDebrisScore)
            leftDebrisServo.setPosition(leftDebrisOut);
        else
            leftDebrisServo.setPosition(leftDebrisIn);
        if (rightDebrisScore)
            rightDebrisServo.setPosition(rightDebrisOut);
        else
            rightDebrisServo.setPosition(rightDebrisIn);

        gamepadY = gamepad1.y;
        // If Y button is pressed (and was not last time), set servo to be updated
        if (gamepadY & !lastY)
            lockBrake = !lockBrake;
        lastY = gamepadY; // Update last button value

        // Update brake servo as necessary
        if (lockBrake)
            brakeServo.setPosition(brakeLift);
        else
            brakeServo.setPosition(releaseLift);

        gamepadA = gamepad1.a;
        if (gamepadA & !lastA)
            zipOut = !zipOut;
        lastA = gamepadA;

        if (zipOut) {
            rightZipServo.setPosition(rightZipOut);
            leftZipServo.setPosition(leftZipOut);
        } else {
            rightZipServo.setPosition(rightZipIn);
            leftZipServo.setPosition(leftZipIn);
        }

        gamepadB = gamepad1.b;
        if(gamepadB & !lastB)
            autoOut = !autoOut;
        lastB = gamepadB;

        if(autoOut) {
            rightAutoServo.setPosition(rightAutoOut);
            leftAutoServo.setPosition(leftAutoOut);
        } else {
            rightAutoServo.setPosition(rightAutoIn);
            leftAutoServo.setPosition(leftAutoIn);
        }

        if(gamepad1.right_stick_button)
            toggleServo.setPosition(toggleOpen);
        else
            toggleServo.setPosition(toggleClose);
    } // loop
} // TeleOp2