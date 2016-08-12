package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.AudioManager;
import android.media.ToneGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor; // gyro
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.Telemetry;



// This was originally my modified PushBotSquare op mode

public class KTestOpMode extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchSensor; // regular touch sensor
    TouchSensor touchSensor2; // legacy touch sensor
    LightSensor led;
    ToneGenerator tg;
    GyroSensor gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        led = hardwareMap.lightSensor.get("legacy_light");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        touchSensor2 = hardwareMap.touchSensor.get("legacy_touch");
        gyro = hardwareMap.gyroSensor.get("sensor_gyro");
        boolean iteration= true;
        waitForStart();
        /*
        for(int i=0; i<1; i++) {
            // code in for loop is mostly changed
            // robot should now drive forward instead of in square
            //leftMotor.setPower(1.0);
            rightMotor.setPower(1.0);
            leftMotor.setPower(0.1);

            sleep(5000);
        }
        */

        // code for sound
        //tg = new ToneGenerator(AudioManager.STREAM_RING, ToneGenerator.MAX_VOLUME);
        //tg.startTone(ToneGenerator.TONE_DTMF_D, 500);
        rightMotor.setPower(1.0);
        leftMotor.setPower(1.0);
        telemetry.addData("gyro", gyro.getRotation());

        while(!touchSensor2.isPressed())
        {
            // display gyro value

            // run motors, flash light sensor

            //led.enableLed(iteration);
            //iteration = !iteration;
        }
        led.enableLed(true); // turn off light sensor

        // stop motors
        leftMotor.setPowerFloat();
        rightMotor.setPowerFloat();

        //rightMotor.setPower(0);
        //leftMotor.setPower(0);

    }
}