package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class TeleOpFunctions {

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

    public final double Kp = .05;

    boolean isStopped;
    GamePadState previousGameStickState;
    GamePadState currentGameStickState;

    AlignState alignState;
    LightState lightState;
    private DistanceSensor sensorRange;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    private BHI260IMU imu;



    SampleMecanumDrive drive;

    public void init(SampleMecanumDrive d, HardwareMap hardwareMap) {
        drive = d;

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        imu = hardwareMap.get(BHI260IMU.class, "IMU");

        BHI260IMU.Parameters myIMUparameters;

        myIMUparameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(myIMUparameters);


        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        previousGameStickState = GamePadState.NETURAL;
        currentGameStickState = GamePadState.NETURAL;
        alignState = AlignState.UNALIGNED;
        lightState = LightState.OFF;
        isStopped = false;
    }

    public void run() {
        switch (alignState) {
            case UNALIGNED:
                if (sensorRange.getDistance(DistanceUnit.CM) < 40) {
                    alignState = AlignState.ALIGNING;
                    lightState = LightState.ALIGNING;
                }
                isStopped = false;
                break;
            case ALIGNING:
                if (sensorRange.getDistance(DistanceUnit.CM) < 8) {
                    alignState = AlignState.ALIGNED;
                    lightState = LightState.ALIGNED;
                } else if (sensorRange.getDistance(DistanceUnit.CM) > 40) {
                    alignState = AlignState.UNALIGNED;
                    lightState = LightState.OFF;
                }
                break;
            case ALIGNED:
                if (sensorRange.getDistance(DistanceUnit.CM) > 8 && sensorRange.getDistance(DistanceUnit.CM) < 40) {
                    alignState = AlignState.ALIGNING;
                    lightState = LightState.ALIGNING;
                } else if (sensorRange.getDistance(DistanceUnit.CM) > 40) {
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
    }

    // positive counter clockwise
    public void turnAlign (GamepadEx gamepad1, double power) {

        double leftYControl = 0 ;
        double leftXControl = 0 ;
        double rightXControl = power;

        while (sensorRange.getDistance(DistanceUnit.CM) > 50 && !gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl,
                            -leftXControl,
                            -rightXControl
                    )
            );
            drive.update();

        }

    }

    // Positive degrees is Counter Clockwise
    public void runTurning(double degrees) {
        double power;
        double error;
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Now use these simple methods to extract each angle
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
        error = degrees - Yaw;
        power = error * Kp;

        while (Yaw < Math.abs(degrees)) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            -power
                    )
            );
        }
        drive.update();

        if (Yaw == degrees) {
            imu.resetYaw();
        }

    }

    public double getYawReading() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }

    public String getAlignState() { return alignState.name(); }
    public String getLightState() { return lightState.name(); }
    
    public double getDistanceReadingCM() { return sensorRange.getDistance(DistanceUnit.CM); }
    public double getDistanceReadingMM() { return sensorRange.getDistance(DistanceUnit.MM); }
    
    public double getGameStickMultiplier() { return gameStickMultiplier; }
}