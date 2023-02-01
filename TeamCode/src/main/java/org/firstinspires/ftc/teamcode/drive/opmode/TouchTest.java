package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TouchTest extends LinearOpMode {
    private enum LightState {
        OFF,
        ALIGNING,
        ALIGNED
    }

    private TouchSensor sensorTouch1;
    private TouchSensor sensorTouch2;
    private DigitalChannel greenLED;
    private DigitalChannel redLED;
    LightState lightState;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorTouch1 = hardwareMap.get(TouchSensor.class, "touch1");
        sensorTouch2 = hardwareMap.get(TouchSensor.class, "touch2");

        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        redLED = hardwareMap.get(DigitalChannel.class, "red");

        lightState = LightState.OFF;

        waitForStart();
        while (opModeIsActive()) {
            switch (lightState) {
                case OFF:
                    greenLED.setState(false);
                    redLED.setState(false);
                    if (sensorTouch1.isPressed() && sensorTouch2.isPressed()) {
                        lightState = LightState.ALIGNED;
                    } else if (sensorTouch1.isPressed() || sensorTouch2.isPressed()) {
                        lightState = LightState.ALIGNING;
                    }
                    break;
                case ALIGNING:
                    greenLED.setState(false);
                    redLED.setState(true);
                    if (!sensorTouch1.isPressed() && !sensorTouch2.isPressed()) {
                        lightState = LightState.OFF;
                    } else if (sensorTouch1.isPressed() && sensorTouch2.isPressed()) {
                        lightState = LightState.ALIGNED;
                    }
                    break;
                case ALIGNED:
                    greenLED.setState(true);
                    redLED.setState(false);
                    if (!sensorTouch1.isPressed() && !sensorTouch2.isPressed()) {
                        lightState = LightState.OFF;
                    }
                    break;
            }
            telemetry.addData("Left", sensorTouch1.isPressed());
            telemetry.addData("Right", sensorTouch2.isPressed());
            telemetry.update();
        }

    }
}
