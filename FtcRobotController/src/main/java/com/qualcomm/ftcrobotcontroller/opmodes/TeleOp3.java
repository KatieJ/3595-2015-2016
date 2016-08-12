package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// General tele-op program for robot:
// Control drive motors and allow reversible front, raise and lower boom, adjust angle of boom
// Will also control debris pick up and scoring at later date

// used for testing parts of robot before state

public class TeleOp3 extends OpMode
{
    // Declaring variables
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, liftMotor1, liftMotor2, brushMotor; // All motors
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo; // All servos
    float xValue; // X Value of joystick
    float yValue; // Y value of joystick


    // NOTE: All of these position will have to be updated before state
    // Declaring doubles (positions for various servos)
    double leftServoOutPos = 1.0; // Position for left autonomous servo
    double leftServoInPos = 0.024; // Position for left autonomous servo
    double rightServoOutPos = 0; // Position for right autonomous servo
    double rightServoInPos = 0.976; // Position for right autonomous servo
    double liftStartPos = 0.12; // Starting position for lift servos
    double liftLowPos = 0.43; // Lowered position for lift servos
    double brakeLift = 0.0; // Braked position for brake servo
    double releaseLift = 0.392; // Released position for brake servo

    // DeclaringliftMotor1.setDirection((DcMotor.Direction.REVERSE); booleans
    boolean leftServoOut = false; // Should left servo be sent to out position?
    boolean rightServoOut = false; // Should right servo be sent to out position?
    boolean negateMotor = true; // Reverse fronts?
    boolean lockBrake = false; // Engage brake?
    boolean lastXValue = false; // Previous value from x button
    boolean lastLeftValue = false; // Previous value from dpad_left
    boolean lastRightValue = false; // Previous value from dpad_right
    boolean lastYValue = false; // Previous value from y button
    boolean gamepadX = false; // Holds value from x button (for less frequent retrieval)
    boolean gamepadY = false; // Holds value from y button (for less frequent retrieval)
    boolean gamepadLeft = false; // Holds value from dpad_left (for less frequent retrieval)
    boolean gamepadRight = false; // Holds value from dpad_right (for less frequent retrieval)

    public void init() // Code to run during initialization
    {
        // Get references to motors
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        brushMotor = hardwareMap.dcMotor.get("brushMotor");

        // Reverse right motors so that all motors go in same direction when given positive power
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        // Reverse lift motors so lift raises when given positive power
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Reverse bursh motor
        brushMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get references to servos
        //leftLiftServo = hardwareMap.servo.get("leftLiftServo");
        //rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        //rightAutoServo = hardwareMap.servo.get("rightAutoServo");
        //leftAutoServo = hardwareMap.servo.get("leftAutoServo");
        //brakeServo = hardwareMap.servo.get("brakeServo");

        // Initialize servos
        //leftLiftServo.setPosition(liftStartPos);
        //rightLiftServo.setPosition(liftStartPos);
        //leftAutoServo.setPosition(leftServoInPos);
        //rightAutoServo.setPosition(rightServoInPos);
        //brakeServo.setPosition(releaseLift);
    }

    @Override
    public void loop () // Main tele-op code
    {
        // When button x is pressed the readings from y axis on the joystick are negated, thus
        // reversing the robot's front
        gamepadX = gamepad1.x; // update variable value

        // If button is pressed (and was not pressed last time), then reverse fronts
        // This is so that if the button is pressed and held for a moment, the fronts don't reverse
        // twice, making the button press ineffective
        if (gamepadX & !lastXValue)
        {
            negateMotor = !negateMotor;
        }

        lastXValue = gamepadX; // Update holder for previous value

        // Retrieve value from joystick y axis and negate it if necessary
        if (negateMotor)
        {
            yValue = -gamepad1.left_stick_y;
        }
        else
        {
            yValue = gamepad1.left_stick_y;
        }

        // Get x value of left joystick
        xValue = gamepad1.left_stick_x;

        // calculate motor power
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

        // Code for gamepad buttons below

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

        if (gamepad1.b)
        {
            brushMotor.setPower(1.0);
        }
        else
        {
            brushMotor.setPower(0.0);
        }

        if (gamepad1.left_bumper) // Rotate lift down when left bumper is pressed
        {
            //rightLiftServo.setPosition(liftStartPos);
            //leftLiftServo.setPosition(liftStartPos);
        }
        else if (gamepad1.left_trigger >= 0.5) // Rotate lift up when left trigger pressed > halfway
        {
            //rightLiftServo.setPosition(liftLowPos);
            //leftLiftServo.setPosition(liftLowPos);
        }

        // Update values here (because they are used multiple times and program would otherwise be
        // slowed
        gamepadLeft = gamepad1.dpad_left;
        gamepadRight = gamepad1.dpad_right;

        // If button is pressed (and was not last time), set servo to be updated
        if (gamepadLeft & !lastLeftValue)
        {
            leftServoOut = !leftServoOut;
        }
        lastLeftValue = gamepadLeft; // Update last button value

        // If button is pressed (and was not last time), set servo to be updated
        if (gamepadRight & !lastRightValue)
        {
            rightServoOut = !rightServoOut;
        }
        lastRightValue = gamepadRight; // Update last button value

        // Update autonomous servos as necessary
        if (leftServoOut)
        {
            //leftAutoServo.setPosition(leftServoOutPos);
        }
        else
        {
            //leftAutoServo.setPosition(leftServoInPos);
        }
        if (rightServoOut)
        {
            //rightAutoServo.setPosition(rightServoOutPos);
        }
        else
        {
            //rightAutoServo.setPosition(rightServoInPos);
        }

        gamepadY = gamepad1.y;
        // If Y button is pressed (and was not last time), set servo to be updated
        if (gamepadY & !lastYValue)
        {
            lockBrake = !lockBrake;
        }
        lastYValue = gamepadY; // Update last button value

        // Update brake servo as necessary
        if (lockBrake)
        {
            //brakeServo.setPosition(brakeLift);
        }
        else
        {
            //brakeServo.setPosition(releaseLift);
        }

    } // loop

} // TeleOp2