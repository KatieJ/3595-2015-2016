package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

// Note: leftMotor1 and rightMotor1 do not have encoders on them, other drive motors do

public class autonomous1 extends LinearOpMode
{
    // declaring variables
    DcMotor leftMotor1, leftMotor2, rightMotor1, rightMotor2, liftMotor1, liftMotor2;
    Servo rightAutoServo, leftAutoServo, leftLiftServo, rightLiftServo, brakeServo;
    double leftServoOutPos = 1.0; // Position for left autonomous servo
    double leftServoInPos = 0.024; // Position for left autonomous servo
    double rightServoOutPos = 0; // Position for right autonomous servo
    double rightServoInPos = 0.976; // Position for right autonomous servo
    double liftStartPos = 0.12; // Starting position for lift servos
    double releaseLift = 0.392;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get references to motors
        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        // Reverse right motors so that all wheels move in same direction when given positive power
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Get references to servos
        rightAutoServo = hardwareMap.servo.get("rightAutoServo");
        leftAutoServo = hardwareMap.servo.get("leftAutoServo");
        leftLiftServo = hardwareMap.servo.get("leftLiftServo");
        rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        brakeServo = hardwareMap.servo.get("brakeServo");

        // setChannelMode is deprecated, need to update these lines when Java updates
        // After looking at other code I can probably change to "setMode", but that's untested
        leftMotor1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftMotor2.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor2.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        // Raise lift (to get bottom off of tiles)
        liftMotor1.setPower(1.0);
        liftMotor2.setPower(1.0);
        sleep(250);
        liftMotor1.setPower(0.0);
        liftMotor2.setPower(0.0);

        // Drive forward to shelter
        leftMotor1.setPower(-0.5);
        rightMotor1.setPower(-0.5);
        leftMotor2.setPower(-1.0);
        rightMotor2.setPower(-1.0);
        sleep(4600);
        leftMotor1.setPower(0.0);
        rightMotor1.setPower(0.0);
        leftMotor2.setPower(0.0);
        rightMotor2.setPower(0.0);

        // Place climbers in shelter
        leftAutoServo.setPosition(leftServoOutPos); // Extend left servo
        rightAutoServo.setPosition(rightServoOutPos); // Extend right servo
        // Initialize all other servos
        leftLiftServo.setPosition(liftStartPos);
        rightLiftServo.setPosition(liftStartPos);
        brakeServo.setPosition(releaseLift);
        sleep(1000); // Wait for autonomous servos to fully extend
        leftAutoServo.setPosition(leftServoInPos); // Retract left servo
        rightAutoServo.setPosition(rightServoInPos); // Retract right servo
    }
}
