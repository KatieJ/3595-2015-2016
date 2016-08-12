package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOpMotorTest extends OpMode {
    public void init() {

    }

    public void loop() {
        long x = System.currentTimeMillis();
         gamepad1.reset();
        if(gamepad1.atRest() == false) {
            //while (x < (x+10000)) {
                telemetry.clearData();
                telemetry.addData("gamepad read", "false");

            //}
        } else {
            telemetry.addData("At rest?", gamepad1.atRest());
        }
    }
}
