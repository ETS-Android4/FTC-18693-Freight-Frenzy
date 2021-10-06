/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Mecanum Drive")
@Disabled
public class MecanumDrive extends OpMode {
    public double Status = 5;
    public boolean mutantGamepad = false;
    public String detectedColor;
    public AndroidSoundPool audio;
    // Declare OpMode members.
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public String detectColor() {
        int colorHSV;
        float hue;
        float sat;
        float val;
        // Convert RGB values to HSV color model.
        // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
        colorHSV = Color.argb(robot.color.alpha(), robot.color.red(), robot.color.green(), robot.color.blue());
        // Get hue.
        hue = JavaUtil.colorToHue(colorHSV);
        //telemetry.addData("Hue", hue);
        // Get saturation.
        sat = JavaUtil.colorToSaturation(colorHSV);
        //telemetry.addData("Saturation", sat);
        // Get value.
        val = JavaUtil.colorToValue(colorHSV);
        //telemetry.addData("Value", val);
        // Use hue to determine if it's red, green, blue, etc..
        if (hue < 30) {
            detectedColor = "Red";
            return "Red";
        } else if (hue < 60) {
            detectedColor = "Orange";
            return "Orange";
        } else if (hue < 90) {
            detectedColor = "Yellow";
            return "Yellow";
        } else if (hue < 150) {
            detectedColor = "Green";
            return "Green";
        } else if (hue < 225) {
            detectedColor = "Blue";
            return "Blue";
        } else if (hue < 350) {
            detectedColor = "Purple";
            return "Purple";
        } else {
            detectedColor = "Not Detected";
            return "Not Detected";
        }
    }

    public void Telemetries() {
        if (Status == 2) {
            telemetry.addData("Status", "Danger! Really Low Voltage");
        } else if (Status == 1) {
            telemetry.addData("Status", "WARNING! Low Voltage");
        } else if (mutantGamepad) {
            telemetry.addData("Status", "Running, Mutant Gamepad Enabled");
        } else {
            telemetry.addData("Status", "Running");
        }
        telemetry.addData("Back Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftRear.getVelocity() / robot.driveVelocity * 100, robot.rightRear.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Front Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftFront.getVelocity() / robot.driveVelocity * 100, robot.rightFront.getVelocity() / robot.driveVelocity * 100);
        //telemetry.addData("Ramp Power", "Bottom (%.2f%%), Middle (%.2f%%), Top (%.2f%%)", robot.rampBottom.getPower() / robot.servoPower * 100, robot.rampMiddle.getPower() / robot.servoPower * 100, robot.rampTop.getPower() / robot.servoPower * 100);
        //telemetry.addData("Claw Power", "Arm (%.2f), Hand (%.2f)", robot.clawArm.getPower() * 100, robot.clawHand.getPower() * 100);
        telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        telemetry.addData("Color Detected", detectColor());

        //telemetry.addData("Temperature","%.2f", robot.gyro.getTemperature().toUnit(TempUnit.FARENHEIT));
    }

/*    public void Drive() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftFront.setPower(v1 * robot.driveVelocity);
        robot.rightFront.setPower(v2 * robot.driveVelocity);
        robot.leftRear.setPower(v3 * robot.driveVelocity);
        robot.rightRear.setPower(v4 * robot.driveVelocity);
    }
    */

    public double m1, m2, m3, m4;
    public void Drive(double x, double y, double r){
        m1 = y+x+r;
        m2 = y-x-r;
        m3 = y+x+r;
        m4 = y-x-r;
        robot.leftFront.setVelocity(m1*robot.driveVelocity);
        robot.rightFront.setVelocity(m2*robot.driveVelocity);
        robot.leftRear.setVelocity(m3*robot.driveVelocity);
        robot.rightRear.setVelocity(m2*robot.driveVelocity);
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap, 2);
        audio = new AndroidSoundPool();

        telemetry.addData("Status", "Initialized");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (robot.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
            }
        }
        telemetry.update();
        Telemetries();
        Drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad2.right_stick_x);

        if (robot.voltageSensor.getVoltage() < robot.reallyLowBattery && Status != 2) {
            if (Status != 1) audio.play("RawRes:ss_siren");
            Status = 2;
            //robot.shootVelocity = robot.maxShootVelocity * 0.75;
            robot.driveVelocity = robot.maxDriveVelocity * 0.5;
        } else if (robot.voltageSensor.getVoltage() < robot.lowBattery && Status != 1) {
            if (Status != 2) audio.play("RawRes:ss_siren");
            Status = 1;
            robot.driveVelocity = robot.maxDriveVelocity * 0.75;
        } else {
            Status = 0;
            //robot.shootVelocity = robot.maxShootVelocity;
            robot.driveVelocity = robot.maxDriveVelocity;
            audio.stop();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    //Stop Code. Runs Once
    public void stop() {
        telemetry.addData("Status", "Stopping...");
        audio.close();
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        telemetry.addData("Status", "Stopped");
        //robot.greenLight.enableLight(false);
        telemetry.update();
    }
}
