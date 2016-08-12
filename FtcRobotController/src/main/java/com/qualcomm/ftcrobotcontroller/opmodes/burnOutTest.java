package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by katiejohnson on 4/21/16.
 */
public class burnOutTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;

        leftDrive1 = hardwareMap.dcMotor.get("leftDrive1");
        leftDrive2 = hardwareMap.dcMotor.get("leftDrive2");
        rightDrive1 = hardwareMap.dcMotor.get("rightDrive1");
        rightDrive2 = hardwareMap.dcMotor.get("rightDrive2");
        leftDrive1.setPowerFloat();
        leftDrive2.setPowerFloat();
        rightDrive1.setPowerFloat();
        rightDrive2.setPowerFloat();
        waitForStart();
        telemetry.addData("Current motor", "leftDrive1");
        leftDrive1.setPower(1.0);
        sleep(1000);
        leftDrive1.setPowerFloat();
        sleep(1000);
        telemetry.clearData();
        telemetry.addData("Current motor", "leftDrive2");
        leftDrive2.setPower(1.0);
        sleep(1000);
        leftDrive2.setPowerFloat();
        sleep(1000);
        telemetry.clearData();
        telemetry.addData("Current motor", "rightDrive1");
        rightDrive1.setPower(1.0);
        sleep(1000);
        rightDrive1.setPowerFloat();
        sleep(1000);
        telemetry.clearData();
        telemetry.addData("Current motor", "rightDrive2");
        rightDrive2.setPower(1.0);
        sleep(1000);
        rightDrive2.setPowerFloat();
    }
}
