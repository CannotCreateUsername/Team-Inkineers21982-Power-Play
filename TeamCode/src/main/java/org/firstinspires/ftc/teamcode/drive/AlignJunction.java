package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class AlignJunction {

    public enum GamePadState {
        POSITIVE,
        NETURAL,
        NEGATIVE
    }

    public enum LightState {
        OFF,
        ALIGNING,
        ALIGNED
    }

    private enum AlignState {
        UNALIGNED,
        ALIGNING,
        ALIGNED
    }

    private double gameStickMultiplier;

    /*
    Time Base Ramping
    If the driver keep on pressing the gamepad, the sensitivity of the gamepad input will increase over time
    */
    ElapsedTime xGamePadTimer;
    boolean isStopped;
    GamePadState previousGameStickState;
    GamePadState currentGameStickState;

    AlignState alignState;
    LightState lightState;
    private DistanceSensor sensorRange;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    public void init(HardwareMap hardwareMap) {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        previousGameStickState = GamePadState.NETURAL;
        currentGameStickState = GamePadState.NETURAL;
        alignState = AlignState.UNALIGNED;
        lightState = LightState.OFF;
        xGamePadTimer = new ElapsedTime();
        isStopped = false;
    }

    public double  getGamepadStickRampingMultiplier(float gameStick) {

        previousGameStickState = currentGameStickState;
        if (gameStick < 0){
            currentGameStickState = GamePadState.NETURAL;
        } else  if (gameStick > 0){
            currentGameStickState = GamePadState.POSITIVE;
        } else {
            currentGameStickState = GamePadState.NETURAL;
        }

        switch (alignState) {
            case UNALIGNED:
                if (sensorRange.getDistance(DistanceUnit.CM) < 50) {
                    alignState = AlignState.ALIGNING;
                    lightState = LightState.ALIGNING;
                }
                isStopped = false;
                gameStickMultiplier = 1;
                break;
            case ALIGNING:
                if (sensorRange.getDistance(DistanceUnit.CM) < 13) {
                    alignState = AlignState.ALIGNED;
                    lightState = LightState.ALIGNED;
                } else if (sensorRange.getDistance(DistanceUnit.CM) > 50) {
                    alignState = AlignState.UNALIGNED;
                    lightState = LightState.OFF;
                }
                gameStickMultiplier = 0.8;
                break;
            case ALIGNED:
                if (sensorRange.getDistance(DistanceUnit.CM) > 8 && sensorRange.getDistance(DistanceUnit.CM) < 50) {
                    alignState = AlignState.ALIGNING;
                    lightState = LightState.ALIGNING;
                } else if (sensorRange.getDistance(DistanceUnit.CM) > 50) {
                    alignState = AlignState.UNALIGNED;
                    lightState = LightState.OFF;
                }
//                isStopped = false;
//                if (currentGameStickState == previousGameStickState && !isStopped) {
//                    gameStickMultiplier = 0;
//                } else if (currentGameStickState != previousGameStickState) {
//                    isStopped = true;
//                    gameStickMultiplier = 1;
//                }
                break;
        }

        switch (lightState) {
            case OFF:
                greenLED.setState(false);
                redLED.setState(false);
                break;
            case ALIGNING:
                greenLED.setState(false);
                redLED.setState(true);
                break;
            case ALIGNED:
                greenLED.setState(true);
                redLED.setState(false);
                break;
        }

        return gameStickMultiplier;

    }

    public String getAlignState() { return alignState.name(); }
    public String getLightState() { return lightState.name(); }
    
    public double getDistanceReadingCM() { return sensorRange.getDistance(DistanceUnit.CM); }
    public double getDistanceReadingMM() { return sensorRange.getDistance(DistanceUnit.MM); }
    
    public double getGameStickMultiplier() { return gameStickMultiplier; }
}