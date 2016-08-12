package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

public class autonomous2 extends LinearOpMode {
    DcMotor leftMotor1;
    DcMotor rightMotor1;
    DcMotor leftMotor2;
    DcMotor rightMotor2;
    Servo rightLiftServo;
    Servo leftLiftServo;
    double leftServoOutPos = 1.0; // Position for left autonomous servo
    double leftServoInPos = 0.024; // Position for left autonomous servo
    double rightServoOutPos = 0; // Position for right autonomous servo
    double rightServoInPos = 0.976; // Position for right autonomous servo

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftMotor1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor1.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitForStart();

        leftMotor1.setPower(-1.0);
        rightMotor1.setPower(-1.0);
        leftMotor2.setPower(-1.0);
        rightMotor2.setPower(-1.0);

        sleep(4000);

        leftMotor1.setPower(0.0);
        rightMotor1.setPower(0.0);
        leftMotor2.setPower(0.0);
        rightMotor2.setPower(0.0);

        leftLiftServo.setPosition(leftServoOutPos);
        rightLiftServo.setPosition(rightServoOutPos);

    }
}
