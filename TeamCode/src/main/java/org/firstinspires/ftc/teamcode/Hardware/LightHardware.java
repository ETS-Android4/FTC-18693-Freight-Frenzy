package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LightHardware {
    public LED[] lights = new LED[16];
    HardwareMap hwMap = null;
    public Boolean initialized = null;


    public void init(HardwareMap ahwMap) {
        initialized = false;
        hwMap = ahwMap;
        for (int i = 0; i < lights.length; i++) {
            lights[i] = hwMap.get(LED.class, "Light_" + i);
        }
        initialized = true;
    }

    public void setLights(boolean enable) {
        for (LED light : lights) {
            light.enable(enable);
        }
    }

    public void setLights(boolean left, boolean right) {
        for (int i = 0; i < lights.length; i++) {
            if (i < lights.length / 2) {
                lights[i].enable(left);
            } else {
                lights[i].enable(right);
            }
        }
    }

    public void setGreenLights(boolean enable) {
        for (int i = 1; i < lights.length; i += 2) {
            lights[i].enable(enable);
        }
    }

    public void setGreenLights(boolean left, boolean right) {
        for (int i = 1; i < lights.length; i += 2) {
            if (i < lights.length / 2) {
                lights[i].enable(left);
            } else {
                lights[i].enable(right);
            }
        }
    }

    public void setRedLights(boolean enable) {
        for (int i = 0; i < lights.length; i += 2) {
            lights[i].enable(enable);
        }
    }

    public void setRedLights(boolean left, boolean right) {
        for (int i = 0; i < lights.length; i += 2) {
            if (i < lights.length / 2) {
                lights[i].enable(left);
            } else {
                lights[i].enable(right);
            }
        }
    }

    public void random(boolean green, boolean red) {
        for (int i = 0; i < lights.length; i += 2) {
            lights[i].enable(red && Math.random() < 0.5);

        }
        for (int i = 1; i < lights.length; i += 2) {
            lights[i].enable(green && Math.random() < 0.5);
        }

    }

    public void random() {
        for (LED light : lights) {
            light.enable(Math.random() < 0.5);
        }
    }
}
