package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by katiejohnson on 4/10/16.
 */
public class autonomousMenu extends LinearOpMode {

    // Button booleans
    boolean right = false;
    boolean left = false;
    boolean lastRight = false;
    boolean lastLeft = false;

    int variable = 0; // generic variable for method to adjust

    int color = 0; // 0 = red, 1 = blue
    int startPos = 0; // 0 = close, 1 = far
    int whichAuto = 0; // 0 = full, 1 = mountain
    int waitTime = 0; // Time to wait (in seconds)
    int endPos = 0; // 0 = beacon repair zone, 1 = corner

    // For time to wait, time the slowest autonomous and do 30/that time, that's the maximum delay
    // Or it could change based on what autonomous was selected

    public void runOpMode() throws InterruptedException {
        while(!gamepad1.b) {
            color = menuItem(2);
            if (color == 0)
                telemetry.addData("Color (change with arrows)", " Red");
            else
                telemetry.addData("Color (change with arrows)", " Blue");
        }
        reset();

        while(!gamepad1.b) {
            startPos = menuItem(2);
            if (startPos == 0)
                telemetry.addData("Start position (change with arrows)", " Close");
            else
                telemetry.addData("Start position (change with arrows)", " Far");
        }
        reset();

        while(!gamepad1.b) {
            whichAuto = menuItem(2);
            if(whichAuto == 0)
                telemetry.addData("Which autonomous? (change with arrows)", "Full");
            else
                telemetry.addData("Which autonomous? (change with arrows)", "Just mountain");
        }
        reset();

        while(!gamepad1.b) {
            waitTime = menuItem(11);
            telemetry.addData("Time to wait (change with arrows)", Integer.toString(waitTime));
        }
        reset();

        if(whichAuto == 0) {
            while (!gamepad1.b) {
                endPos = menuItem(2);
                if (endPos == 0)
                    telemetry.addData("End position (change with arrows)", "Beacon repair zone");
                else
                    telemetry.addData("End position (change with arrows)", "Corner/opposing side");
            }
            reset();
        }

        while(!gamepad1.b) {
            telemetry.addData("Press \"b\" to calibrate gyro", "");
        }
        // calibrate gyro here
        telemetry.clearData();

        displayChoices();

        waitForStart();
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
