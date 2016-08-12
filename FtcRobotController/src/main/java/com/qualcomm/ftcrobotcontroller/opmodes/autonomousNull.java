// Null autonomous (initializes into the cube, but doesn't do anything else)

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class autonomousNull extends LinearOpMode
{
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;
    Servo leftLiftServo, rightLiftServo, rightAutoServo, leftAutoServo, brakeServo,
            rightDebrisServo, leftDebrisServo, rightZipServo, leftZipServo, toggleServo,
            leftFenderServo, rightFenderServo;

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
    public void runOpMode() throws InterruptedException
    {
        // Get references to motors
        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");

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

        waitForStart();
    }
}
