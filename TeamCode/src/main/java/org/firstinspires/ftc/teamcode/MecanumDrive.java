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

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.Hardware.GyroHardware;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

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
//@Disabled
public class MecanumDrive extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public double Status = 5;
    //public boolean mutantGamepad = false;
    public boolean worldDrive = false;
    public String detectedColor;
    public AndroidSoundPool audio;
    public double steeringMultiplier = 1;
    public double steeringAdjusted = 0;
    public double driveModeAdjusted = 0;
    public double m1, m2, m3, m4, g;
    public double maxDrive;
    public double minDrive;
    // Declare OpMode members.
    boolean opModeIsActive = false;
    RobotHardware robot = new RobotHardware();
    CameraHardware camera = new CameraHardware();
    GyroHardware gyro = new GyroHardware();
    Thread initialization = new Thread(() -> {
        robot.init(hardwareMap, 2);
        telemetry.speak("Hardware Online");
        camera.init(hardwareMap, 1);
        telemetry.speak("Camera Online");
        gyro.init(hardwareMap);
        telemetry.speak("Gyro Online");
    });
    Thread user1 = new Thread(() -> {

    });
    Thread user2 = new Thread(() -> {

    });
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    int lei = 0;

    public String detectColor() {
        int colorHSV;
        float hue;
        //float sat;
        //float val;
        // Convert RGB values to HSV color model.
        // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
        colorHSV = Color.argb(robot.color.alpha(), robot.color.red(), robot.color.green(), robot.color.blue());
        // Get hue.
        hue = JavaUtil.colorToHue(colorHSV);
        //telemetry.addData("Hue", hue);
        // Get saturation.
        //sat = JavaUtil.colorToSaturation(colorHSV);
        //telemetry.addData("Saturation", sat);
        // Get value.
        //val = JavaUtil.colorToValue(colorHSV);
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

    @SuppressLint("DefaultLocale")
    public void Telemetries() {
        if (!robot.initialized || !gyro.initialized || !camera.initialized) {
            telemetry.addData("Status", "Initializing...");
            if (!robot.initialized)
                telemetry.addData("Hardware", "Initializing...");
            if (!camera.initialized)
                telemetry.addData("Camera", "Initializing...");
            if (!gyro.initialized)
                telemetry.addData("Gyro", "Initializing...");
        } else if (Status == 2) {
            telemetry.addData("Status", "Danger! Really Low Voltage");
        } else if (Status == 1) {
            telemetry.addData("Status", "WARNING! Low Voltage");
        } else if (worldDrive) {
            telemetry.addData("Status", "Running, World Drive Enabled");
        } else {
            telemetry.addData("Status", "Running");
        }

        /*if (camera.initialized) {
            camera.findObject();
            for (int j = 0; j < camera.CamLeft.length; j++) {
                int i = camera.ObjectNum;
                telemetry.addData("# Object Detected", camera.ObjectsDetected);
                telemetry.addData(String.format("label (%d)", i), camera.ObjectName);
                telemetry.addData(String.format("  left,top (%d)", i), "%d , %d",
                        camera.CamLeft[i], camera.CamTop[i]);
                telemetry.addData(String.format("  right,bottom (%d)", i), "%d , %d",
                        camera.CamRight[i], camera.CamBottom[i]);
            }
        }
        */
        if (gyro.initialized) {
            telemetry.addData("Intrinsic Orientation", "%.0f°", gyro.getOrientation().thirdAngle);
            telemetry.addData("Extrinsic Orientation", "%.0f°", gyro.getOrientation2().thirdAngle);
            telemetry.addData("Temperature", "%.0f°", gyro.getTemp() * 1.8 + 32);
        }
        // 1,500 = up, 0 = down
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Steering Sensitivity", "%.0f%%", steeringMultiplier * 100);
        telemetry.addData("Front Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftFront.getVelocity() / robot.driveVelocity * 100, robot.rightFront.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Rear Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftRear.getVelocity() / robot.driveVelocity * 100, robot.rightRear.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        telemetry.addData("Color Detected", detectColor());

        if (camera.initialized) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = camera.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.01f , %.01f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.01f , %.01f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
            }
        }
    }

    public void Drive(double x, double y, double z) {
        if (gamepad1.left_bumper) {
            maxDrive = 0.5;
            minDrive = -0.5;
        } else if (gamepad1.right_bumper) {
            maxDrive = 1;
            minDrive = -1;
        } else {
            maxDrive = 0.75;
            minDrive = -0.75;
        }
        //   r *= steeringMultiplier;
        m1 = Range.clip(y + x + z * steeringMultiplier, minDrive, maxDrive);
        m2 = Range.clip(y - x - z * steeringMultiplier, minDrive, maxDrive);
        m3 = Range.clip(y - x + z * steeringMultiplier, minDrive, maxDrive);
        m4 = Range.clip(y + x - z * steeringMultiplier, minDrive, maxDrive);
        robot.leftFront.setVelocity(m1 * robot.driveVelocity);
        robot.rightFront.setVelocity(m2 * robot.driveVelocity);
        robot.leftRear.setVelocity(m3 * robot.driveVelocity);
        robot.rightRear.setVelocity(m4 * robot.driveVelocity);
    }

    public void WorldDrive(double x, double y, double z, Orientation gyro) {
        g = gyro.thirdAngle / 90;
        m1 = Range.clip((y + x - g) + z * steeringMultiplier, -1, 1);
        m2 = Range.clip((y - x + g) - z * steeringMultiplier, -1, 1);
        m3 = Range.clip((y - x + g) + z * steeringMultiplier, -1, 1);
        m4 = Range.clip((y + x - g) - z * steeringMultiplier, -1, 1);
        robot.leftFront.setVelocity(m1 * robot.driveVelocity);
        robot.rightFront.setVelocity(m2 * robot.driveVelocity);
        robot.leftRear.setVelocity(m3 * robot.driveVelocity);
        robot.rightRear.setVelocity(m4 * robot.driveVelocity);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //initialization.setPriority(9);
        initialization.start();
        audio = new AndroidSoundPool();


    }

    @Override
    public void init_loop() {
    /*    if(lei >15) lei = 0;
        robot.lights[lei].enable(true);
        if (lei < 1) robot.lights[15].enable(false);
        else robot.lights[lei - 1].enable(false);
        //robot.light2.enableLight((int)runtime.seconds() % 2 == 1);
        lei++;
        sleep(200);

     */
        telemetry.addData("Status", (robot.initialized && gyro.initialized && camera.initialized) ? "Initialized, Ready For Start" : "Initializing...");
        telemetry.addData("Hardware", robot.initialized ? "Initialized" : "Initializing...");
        telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
        telemetry.addData("Gyro", gyro.initialized ? "Initialized" : "Initializing...");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        opModeIsActive = true;
        while(!robot.initialized){
            telemetry.addData("Status", "Initializing...");
            telemetry.addData("Hardware", robot.initialized ? "Initialized" : "Initializing...");
            telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
            telemetry.addData("Gyro", gyro.initialized ? "Initialized" : "Initializing...");
        }
        // robot.light2.enable(false);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        for (int i = 0; i < robot.lights.length; i++) {
            robot.lights[i].enable(true);
            if (i < 1) robot.lights[15].enable(false);
            else robot.lights[i - 1].enable(false);
            // while(!robot.lights[1].isLightOn())
        }
        //robot.light1.enableLight((int)runtime.seconds() % 2 == 1);
        //robot.light1.enableLight(true);
        telemetry.update();
        Telemetries();
        if (gamepad1.b) gyro.gyro.initialize(gyro.parameters);
        if (gamepad1.a && driveModeAdjusted < runtime.milliseconds()) {
            //worldDrive = true;
            driveModeAdjusted = runtime.milliseconds() + 500;
        } else if (worldDrive) {
            WorldDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gyro.gyro.getAngularOrientation());
        } else {
            Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        if (gamepad1.back && steeringAdjusted < runtime.milliseconds()) {
            steeringMultiplier -= 0.1;
            steeringMultiplier = Range.clip(steeringMultiplier, 0, 2);
            steeringAdjusted = runtime.milliseconds() + 100;
        } else if (gamepad1.start && steeringAdjusted < runtime.milliseconds()) {
            steeringMultiplier += 0.1;
            steeringMultiplier = Range.clip(steeringMultiplier, 0, 2);
            steeringAdjusted = runtime.milliseconds() + 100;
        }
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
        opModeIsActive = false;
        telemetry.addData("Status", "Stopping...");
        audio.close();
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        if (camera.tfod != null) camera.tfod.shutdown();
        telemetry.addData("Status", "Stopped");
        //robot.greenLight.enableLight(false);

    }
}
