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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    public AndroidSoundPool audio = new AndroidSoundPool();
    public double steeringMultiplier = 1;
    public double steeringAdjusted = 0;
    public double driveModeAdjusted = 0;
    public double m1, m2, m3, m4, g;
    public double maxDrive;
    public double minDrive;
    public boolean positionSaved = false;
    public double lastPosition = 0;
    private boolean LEDOveride = false;
    // Declare OpMode members.
    double gamepad2ATime = 0;
    boolean gamepad2AReleased = true;
    double gamepad1GuideTime = 0;
    boolean gamepad1GuideReleased = true;

    boolean opModeIsActive = false;
    RobotHardware robot = new RobotHardware();
    CameraHardware camera = new CameraHardware();
    GyroHardware gyro = new GyroHardware();
    Thread initialization = new Thread(() -> {
        robot.init(hardwareMap, 2);
        robot.setLights(false);
        robot.setGreenLights(true);
        telemetry.speak("Hardware Online");
        camera.init(hardwareMap, 1);
        if (camera.initialized != null) {
            telemetry.speak("Camera Online");
        }/* else {
            telemetry.speak("Camera Offline");
        }*/
        gyro.init(hardwareMap);
        telemetry.speak("Gyroscope Online");
    });
    /*  Thread user1 = new Thread(() -> {
          while (opModeIsActive) {
              //if (gamepad1.b) gyro.gyro.initialize(gyro.parameters);
              /*if (gamepad1.a && driveModeAdjusted < runtime.milliseconds()) {
                  //worldDrive = true;
                  driveModeAdjusted = runtime.milliseconds() + 500;
              } else if (worldDrive) {
                  WorldDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gyro.gyro.getAngularOrientation());
              } else {
                  Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
              }

              //Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
              if (gamepad1.back && steeringAdjusted < runtime.milliseconds()) {
                  steeringMultiplier -= 0.1;
                  steeringMultiplier = Range.clip(steeringMultiplier, 0, 2);
                  steeringAdjusted = runtime.milliseconds() + 100;
              } else if (gamepad1.start && steeringAdjusted < runtime.milliseconds()) {
                  steeringMultiplier += 0.1;
                  steeringMultiplier = Range.clip(steeringMultiplier, 0, 2);
                  steeringAdjusted = runtime.milliseconds() + 100;
              }
          }
      });
  */
    /*Thread user2 = new Thread(() -> {
        while (opModeIsActive) {
            if (gamepad2.left_bumper) {
                UnlockMotor(robot.arm, true);
            } else if (robot.arm.getCurrentPosition() < robot.armMin) {
                if (-gamepad2.left_stick_y > 0) robot.arm.setVelocity(-gamepad2.left_stick_y*robot.armVelocity);
                else if (-gamepad2.left_stick_y < 0){
                    UnlockMotor(robot.arm, true);
                } else {
                    LockMotor(robot.arm);
                }
            } else if (robot.arm.getCurrentPosition() > robot.armMax) {
                if (-gamepad2.left_stick_y < 0) robot.arm.setVelocity(-gamepad2.left_stick_y*robot.armVelocity);
                else LockMotor(robot.arm);
            } else if (gamepad2.left_stick_y != 0) {
                robot.arm.setVelocity(-gamepad2.left_stick_y*robot.armVelocity);
                positionSaved = false;
            } else {
                LockMotor(robot.arm);
            }
            if(gamepad2.right_bumper && gamepad2AReleased && runtime.seconds()-gamepad2ATime>0.2){
                gamepad2AReleased = false;
                robot.claw.setPosition(robot.claw.getPosition() == 0 ? 1 : 0);
            } else if (!gamepad2AReleased && !gamepad2.a){
                gamepad2ATime = runtime.seconds();
                gamepad2AReleased = true;
            }
            //robot.spinner.setPower(gamepad2.right_trigger > 0 ? gamepad2.right_trigger : gamepad2.left_trigger);
        }
    });
     */

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void LockMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!positionSaved) {
            lastPosition = motor.getCurrentPosition() > robot.armMax ? robot.armMax : motor.getCurrentPosition();
            positionSaved = true;
        }
        //motor.setPower((lastPosition - motor.getCurrentPosition()) / 100);
        //motor.setPower(lastPosition - motor.getCurrentPosition()/100.0);
        //motor.setPower(0.2);
        double correction = lastPosition - motor.getCurrentPosition();
        correction += correction == 0 ? 1 : 0;
        //motor.setPower(Range.clip(correction / robot.armMax, correction <= -10 ? -0.2 : 0.01, 0.5));
        motor.setPower(Range.clip(correction / 10, -0.5, 0.75));
    }

    public void UnlockMotor(DcMotor motor, boolean powerOff) {
        if (powerOff) motor.setPower(0);
        positionSaved = false;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

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
            telemetry.addData("Orientation", "%.0f°", -gyro.getOrientation().secondAngle);
            telemetry.addData("Temperature", "%.0f°", gyro.getTemp() * 1.8 + 32);
        }
        // 1,500 = up, 0 = downG
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        if (positionSaved)
            telemetry.addData("Arm Accuracy", lastPosition - robot.arm.getCurrentPosition());
        telemetry.addData("Arm Power", "%.3f", robot.arm.getPower());
        telemetry.addData("Servo Position", "%.2f", robot.claw.getPosition());
        telemetry.addData("Front Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftFront.getVelocity() / robot.driveVelocity * 100, robot.rightFront.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Rear Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftRear.getVelocity() / robot.driveVelocity * 100, robot.rightRear.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Steering Sensitivity", "%.0f%%", steeringMultiplier * 100);
        //telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        //telemetry.addData("Color Detected", robot.detectColor());
        if (camera.initialized != null && camera.initialized) {
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

    boolean r2d2 = false;

    public void Drive(double x, double y, double z) {
        if (gamepad1.left_bumper) {
            maxDrive = 0.25;
            minDrive = -0.25;
            if (gamepad1.isRumbling())
                gamepad1.stopRumble();
            LEDOveride = true;
            robot.setGreenLights(true);
            robot.setRedLights(false);
        } else if (gamepad1.right_bumper) {
            if (!r2d2 && x+y+z != 0) {
                audio.play("1-screaming.mp3");
                r2d2 = true;
            }
            maxDrive = 1;
            minDrive = -1;
            if (!gamepad1.isRumbling())
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            LEDOveride = true;
            robot.setRedLights(true);
            robot.setGreenLights(false);
        } else {
            r2d2 = false;
            maxDrive = 0.5;
            minDrive = -0.5;
            if (gamepad1.isRumbling())
                gamepad1.stopRumble();
            LEDOveride = false;

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

    double lastOrientation = 2;

    public void WorldDrive(double x, double y, double z) {
        g = -gyro.getOrientation().secondAngle / 45;
        if ((lastOrientation > 160 || lastPosition < -160) && (gyro.getOrientation().secondAngle < 20 || gyro.getOrientation().secondAngle < -20))
            g = 4;
        lastOrientation = -gyro.getOrientation().secondAngle;
        if (g > 2 || g < -2) {
            g = g > 2 ? -g + 4 : -g - 4;
            Drive((x + g * y) * -1, (y - g * x) * -1, z + 0);
        } else {
            Drive(x - g * y, y + g * x, z + 0);
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initialization.setPriority(7);
        initialization.start();
        audio.initialize(SoundPlayer.getInstance());
        //telemetry.setMsTransmissionInterval(5);
        //user1.setPriority(10);
        //user2.setPriority(8);
        //lights.setPriority(5);


    }

    @Override
    public void init_loop() {
        if (robot.initialized != null && gyro.initialized != null && camera.initialized != null)
            telemetry.addData("Status", (robot.initialized && gyro.initialized && camera.initialized) ? "Initialized, Ready For Start" : "Initializing...");
        else telemetry.addData("Status", "Initializing...");

        if (robot.initialized != null) {
            telemetry.addData("Hardware", robot.initialized ? "Initialized" : "Initializing...");
        } else
            telemetry.addData("Hardware", "Uninitialized");

        if (camera.initialized != null) {
            telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
        } else {
            telemetry.addData("Camera", "Uninitialized");

        }

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
        runtime.reset();
        opModeIsActive = true;
        /*if (robot.initialized == null) robot.initialized = false;
        while (!robot.initialized) {
            telemetry.addData("Hardware", "Initializing...");

            if (camera.initialized != null) {
                telemetry.addData("Camera", camera.initialized ? "Initialized" : "Initializing...");
            } else {
                telemetry.addData("Camera", "Uninitialized");

            }

            if (gyro.initialized != null) {
                telemetry.addData("Gyro", gyro.initialized ? "Initialized" : "Initializing...");
            } else {
                telemetry.addData("Gyro", "Uninitialized");
            }
        }
         */
        try {
            initialization.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //user1.start();
        //user2.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double celebrateTime = 0;

    @Override
    public void loop() {
        if (gamepad1.a || gamepad2.a) {
            if (runtime.seconds() - celebrateTime > 1) {
                //telemetry.speak("WAHOOOOO!!!");
                //audio.play("RawRes:ss_laser_burst");
                celebrateTime = runtime.seconds();
            }
            for (LED light : robot.lights) {
                light.enable(Math.random() < 0.5);
            }
            //robot.setGreenLights(Math.random() < 0.5, Math.random() < 0.5);
            //robot.setRedLights(Math.random() < 0.5, Math.random() < 0.5);
        } else if (!LEDOveride && worldDrive) {
            robot.setGreenLights((int) (runtime.milliseconds() / 500) % 2 == 0, !((int) (runtime.milliseconds() / 500) % 2 == 0));
            robot.setRedLights(!((int) (runtime.milliseconds() / 500) % 2 == 0), (int) (runtime.milliseconds() / 500) % 2 == 0);
        } else if (!LEDOveride) {
            robot.setGreenLights((int) (runtime.milliseconds() / 500) % 2 == 0);
            robot.setRedLights(!((int) (runtime.milliseconds() / 500) % 2 == 0));
        }
        if (worldDrive) {
            WorldDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        if (gamepad1.guide && gamepad1GuideReleased && runtime.seconds() - gamepad1GuideTime > 0.2) {
            gamepad1.stopRumble();
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            robot.setGreenLights(false);
            robot.setRedLights(true);
            worldDrive = !worldDrive;
            telemetry.speak(String.format("%s World Drive", worldDrive ? "Activating" : "Deactivating"));
            if (worldDrive) {
                Drive(0, 0, 0);
                gyro.init();
            }
            gamepad1.stopRumble();
            gamepad1GuideReleased = false;
            robot.setGreenLights(true);
            robot.setRedLights(false);
        } else if (!gamepad1GuideReleased && !gamepad1.guide) {
            gamepad1GuideTime = runtime.seconds();
            gamepad1GuideReleased = true;
        }

        /*for (int i = 0; i < robot.lights.length; i++) {
            robot.lights[i].enable(true);
            if (i < 1) robot.lights[15].enable(false);
            else robot.lights[i - 1].enable(false);
            // while(!robot.lights[1].isLightOn())
        }

         */

        Telemetries();
        if (robot.voltageSensor.getVoltage() < robot.reallyLowBattery && Status != 2) {
            //if (Status != 1) audio.play("RawRes:ss_siren");
            Status = 2;
            //robot.shootVelocity = robot.maxShootVelocity * 0.75;
            robot.driveVelocity = robot.maxDriveVelocity * 0.5;
        } else if (robot.voltageSensor.getVoltage() < robot.lowBattery && Status != 1) {
            //if (Status != 2) audio.play("RawRes:ss_siren");
            Status = 1;
            robot.driveVelocity = robot.maxDriveVelocity * 0.75;
        } else {
            Status = 0;
            //robot.shootVelocity = robot.maxShootVelocity;
            robot.driveVelocity = robot.maxDriveVelocity;
            //audio.stop();
        }
        if (gamepad2.left_bumper) {
            UnlockMotor(robot.arm, true);
        }else if (gamepad2.dpad_down){
            robot.arm.setPower(-1);
        } else if (gamepad2.dpad_up){
            robot.arm.setPower(1);
        }else if (gamepad2.left_stick_y != 0 && gamepad2.guide) {
            robot.arm.setPower(-gamepad2.left_stick_y);
            positionSaved = false;
        } else if (robot.arm.getCurrentPosition() > robot.armMax - 5) {
            if (-gamepad2.left_stick_y < 0)
                robot.arm.setPower(-gamepad2.left_stick_y);
            else LockMotor(robot.arm);
        } else if (robot.arm.getCurrentPosition() < robot.armMin) {
            if (-gamepad2.left_stick_y > 0)
                robot.arm.setPower(-gamepad2.left_stick_y);
            else UnlockMotor(robot.arm, true);
        } else if (gamepad2.left_stick_y != 0) {
            robot.arm.setPower(-gamepad2.left_stick_y);
            positionSaved = false;
        } else {
            LockMotor(robot.arm);
        }
        if (gamepad2.right_trigger > 0) {
            robot.claw.setPosition(gamepad2.right_trigger);
        } else if (gamepad2.right_bumper && gamepad2AReleased && runtime.seconds() - gamepad2ATime > 0.2) {
            gamepad2AReleased = false;
            robot.claw.setPosition(robot.claw.getPosition() == 1 ? 0 : 1);
        } else if (!gamepad2AReleased && !gamepad2.a) {
            gamepad2ATime = runtime.seconds();
            gamepad2AReleased = true;
        }
        robot.spinner.setPower(-gamepad2.right_stick_y);

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
        UnlockMotor(robot.arm, true);
        if (camera.initialized != null && camera.initialized) camera.tfod.shutdown();
        telemetry.addData("Status", "Stopped");
        robot.setLights(false);
        //robot.greenLight.enableLight(false);

    }
}
