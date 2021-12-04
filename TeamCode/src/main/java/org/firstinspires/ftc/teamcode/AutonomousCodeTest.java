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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.GyroHardware;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


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

@Autonomous(name = "Autonomous Program TEST", preselectTeleOp = "Mecanum Drive")
//@Disabled
public class AutonomousCodeTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public double Status = 5;
    //public boolean mutantGamepad = false;
    public boolean worldDrive = false;
    public AndroidSoundPool audio;
    public double steeringMultiplier = 1;
    public double m1, m2, m3, m4;
    public double maxDrive;
    public double minDrive;
    public boolean Done;
    // Declare OpMode members.
    boolean opModeIsActive = false;
    RobotHardware robot = new RobotHardware();
    //CameraHardware camera = new CameraHardware();
    GyroHardware gyro = new GyroHardware();
    Thread initialization = new Thread(() -> {
        robot.init(hardwareMap, 2);
        robot.setLights(false);
        robot.setGreenLights(true);
        telemetry.speak("Hardware Online");
        //camera.init(hardwareMap, 1);
        /*if (camera.initialized != null) {
            telemetry.speak("Camera Online");
        } else {
            sleep(500);
            telemetry.speak("Camera Offline");
        }*/
        gyro.init(hardwareMap);
        telemetry.speak("Gyroscope Online");
    });
    Thread MainPrgm = new Thread(() -> {
        Drive(0, 0.5, 0, 300);
        Drive(1, 0, 0, 100);
    });
    Thread lights = new Thread(() -> {
        robot.setLights(false);
        while (opModeIsActive) {
            for (int i = 0; i < robot.lights.length; i++) {
                if (!opModeIsActive) break;
                if (i < 1) robot.lights[robot.lights.length - 1].enable(false);
                robot.lights[i].enable(true);
                sleep(200);
            }
        }
    });
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @SuppressLint("DefaultLocale")
    public void Telemetries() {
        /*if (!robot.initialized || !gyro.initialized || !camera.initialized) {
            telemetry.addData("Status", "Initializing...");
            if (!robot.initialized)
                telemetry.addData("Hardware", (robot.initialized == null) ? "Uninitialized" : "Initializing...");
            if (!camera.initialized)
                telemetry.addData("Camera", (camera.initialized == null) ? "Uninitialized" : "Initializing...");
            if (!gyro.initialized)
                telemetry.addData("Gyro", (gyro.initialized == null) ? "Uninitialized" : "Initializing...");
        } else */
        if (Status == 2) {
            telemetry.addData("Status", "Danger! Really Low Voltage");
        } else if (Status == 1) {
            telemetry.addData("Status", "WARNING! Low Voltage");
        } else if (worldDrive) {
            telemetry.addData("Status", "Running, World Drive Enabled");
        } else {
            telemetry.addData("Status", "Running");
        }
        if (gyro.initialized != null && gyro.initialized) {
            telemetry.addData("Intrinsic Orientation", "%.0f°", gyro.getOrientation().thirdAngle);
            telemetry.addData("Extrinsic Orientation", "%.0f°", gyro.getOrientation2().thirdAngle);
            telemetry.addData("Temperature", "%.0f°", gyro.getTemp() * 1.8 + 32);
        }
        // 1,500 = up, 0 = downG
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        //telemetry.addData("Motor Position", "LeftRear (%.2f), RightRear (%.2f), LeftFront (%.2f), RightFront (%.2f)",robot.leftRear.getCurrentPosition(), robot.rightRear.getCurrentPosition(), robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
        telemetry.addData("Servo Position", "%.2f", robot.claw.getPosition());
        telemetry.addData("Steering Sensitivity", "%.0f%%", steeringMultiplier * 100);
        telemetry.addData("Front Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftFront.getVelocity() / robot.driveVelocity * 100, robot.rightFront.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Rear Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftRear.getVelocity() / robot.driveVelocity * 100, robot.rightRear.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        //telemetry.addData("Color Detected", robot.detectColor());
        /*if (camera.initialized != null && camera.initialized) {
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
        }*/
    }

    public void Drive(double x, double y, double z, int targetPos) {
        telemetry.addData("xyz", "X, Y, Z, Pos", x, y, z, targetPos);
        while (!Done) {
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
            if (x == 0 && z == 0 && y > 0) {
                if (robot.leftFront.getCurrentPosition() >= targetPos && robot.rightRear.getCurrentPosition() >= targetPos && robot.leftRear.getCurrentPosition() >= targetPos && robot.rightFront.getCurrentPosition() >= targetPos) {
                    Done = true;
                    robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            if (y == 0 && z == 0 && x < 0) {
                if (robot.leftFront.getCurrentPosition() <= -targetPos && robot.rightRear.getCurrentPosition() <= -targetPos && robot.leftRear.getCurrentPosition() >= targetPos && robot.rightFront.getCurrentPosition() >= targetPos) {
                    Done = true;
                    robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setVelocity(0);
                    robot.leftRear.setVelocity(0);
                    robot.rightFront.setVelocity(0);
                    robot.rightRear.setVelocity(0);

                }
            }
            if (x == 0 && z == 0 && y < 0) {
                if (robot.leftFront.getCurrentPosition() <= -targetPos && robot.rightRear.getCurrentPosition() <= -targetPos && robot.leftRear.getCurrentPosition() <= -targetPos && robot.rightFront.getCurrentPosition() <= -targetPos) {
                    Done = true;
                    robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            if (y == 0 && z == 0 && x > 0) {
                if (robot.leftFront.getCurrentPosition() >= targetPos && robot.rightRear.getCurrentPosition() >= targetPos && robot.leftRear.getCurrentPosition() <= -targetPos && robot.rightFront.getCurrentPosition() <= -targetPos) {
                    Done = true;
                    robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        audio = new AndroidSoundPool();
        initialization.start();


    }

    @Override
    public void init_loop() {
        if (robot.initialized != null && gyro.initialized != null/* && camera.initialized != null*/)
            telemetry.addData("Status", (robot.initialized && gyro.initialized/* && camera.initialized*/) ? "Initialized, Ready For Start" : "Initializing...");
        else telemetry.addData("Status", "Initializing...");

        if (robot.initialized != null) {
            telemetry.addData("Hardware", robot.initialized ? "Initialized" : "Initializing...");
        } else
            telemetry.addData("Hardware", "Uninitialized");

        /*if (camera.initialized != null) {
            telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
        } else {
            telemetry.addData("Camera", "Uninitialized");

        }*/

        if (gyro.initialized != null) {
            telemetry.addData("Gyro", gyro.initialized ? "Initialized" : "Initializing...");
        } else {
            telemetry.addData("Gyro", "Uninitialized");
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        opModeIsActive = true;
        if (robot.initialized == null) robot.initialized = false;
        while (!robot.initialized) {
            telemetry.addData("Hardware", "Initializing...");

            /*if (camera.initialized != null) {
                telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
            } else {
                telemetry.addData("Camera", "Uninitialized");

            }

            if (gyro.initialized != null) {
                telemetry.addData("Gyro", gyro.initialized ? "Initialized" : "Initializing...");
            } else {
                telemetry.addData("Gyro", "Uninitialized");
            }*/
        }
        lights.start();
        MainPrgm.start();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Telemetries();
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
        robot.setLights(false);
        telemetry.addData("Status", "Stopping...");
        audio.close();
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        //if (camera.initialized) camera.tfod.shutdown();
        telemetry.addData("Status", "Stopped");
        //robot.greenLight.enableLight(false);

    }
}
