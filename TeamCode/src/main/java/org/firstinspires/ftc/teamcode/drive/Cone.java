package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystemAuto;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Cone {

    private enum PickupState {
        ALIGNING,
        ALIGNED,
        LOADED
    }

    private enum DropOffState {
        ALIGNING,
        ALIGNED,
        UNLOADED
    }

    private enum LightState {
        OFF,
        ALIGNING,
        ALIGNED
    }

    ElapsedTime timer = new ElapsedTime();
    boolean loaded = true;

    // Hardware
    private DistanceSensor sensorRange;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    PickupState pickupState;
    DropOffState dropOffState;
    LightState lightState;

    IntakeSlideSubsystemAuto intakeSlide;
    SampleMecanumDrive drive;

    public void init (SampleMecanumDrive d, IntakeSlideSubsystemAuto i, HardwareMap hardwareMap) {
        drive = d;
        intakeSlide = i;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        pickupState = PickupState.ALIGNING;
        dropOffState = DropOffState.ALIGNING;
        lightState = LightState.OFF;
    }

//    public void init(HardwareMap hardwareMap) {
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        // Distance Sensor
//        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
//
//        slides = hardwareMap.get(DcMotor.class, "slides");
//        slides.setDirection(DcMotor.Direction.REVERSE);
//        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // initialize the hardware map of intake
//        intake = hardwareMap.get(CRServo.class, "intake");
//
//
//        currentCaption = "Lift Status";
//        currentStatus = "Initialized";
//        currentTarget = 0;
//        currentPower = 0;
//
//        intakeState = IntakeSlide.IntakeState.STOP;
//        liftState = IntakeSlide.LiftState.REST;
//        pickupState = PickupState.ALIGNING;
//
//
//    }
//
//    public void run() {
//        switch (liftState) {
//            case REST:
//                currentTarget = targetPositionRest;
//                runToPosition(currentTarget);
//                break;
//            case PICKUP2:
//                currentTarget = targetPositionPickup2 + 400;
//                runToPosition(currentTarget);
//                break;
//            case LOW:
//                currentTarget = targetPositionLow;
//                runToPosition(currentTarget);
//                break;
//            case MEDIUM:
//                currentTarget = targetPositionMedium;
//                runToPosition(currentTarget);
//                break;
//            case HIGH:
//                currentTarget = targetPositionHigh;
//                runToPosition(currentTarget);
//                break;
//        }
//    }
//
//    public void runIntake() {
//        switch (intakeState) {
//            case STOP:
//                setIntakePower(intakeState.STOP);
//                break;
//            case IN:
//                setIntakePower(intakeState.IN);
//                break;
//            case OUT:
//                setIntakePower(intakeState.OUT);
//                break;
//        }
//    }

//    // variables to make typing more efficient
//    private int currentTarget = intakeSlide.getCurrentTarget();
//
//    // PROBLEM: YOU HAVE TO CALL RUN FOR INTAKE SLIDES AND INIT GAMEPADS!! HOW TO SOLVE!?
//    public void pickupCone() {
//        switch (pickupState) {
//            case ALIGNING:
//                lightState = LightState.ALIGNING;
//                currentTarget = intakeSlide.targetPositionPickup2+400;
//                intakeSlide.runToPosition(currentTarget);
//                while (sensorRange.getDistance(DistanceUnit.CM) > 10) {
//                    driveStraight(-0.3);
//                }
//                if (8 < sensorRange.getDistance(DistanceUnit.CM) && sensorRange.getDistance(DistanceUnit.CM) > 10) {
//                    pickupState = PickupState.ALIGNED;
//                }
//                break;
//            case ALIGNED:
//                lightState = LightState.ALIGNED;
//                while (intakeSlide.getSlidePosition() < currentTarget) {
//                    currentTarget = intakeSlide.targetPositionRest+400;
//                    intakeSlide.runToPosition(currentTarget);
//                    intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.IN);
//                }
//                if (intakeSlide.getSlidePosition() > currentTarget-2 && intakeSlide.getSlidePosition() < currentTarget+2) {
//                    pickupState = PickupState.LOADED;
//                }
//                break;
//            case LOADED:
//                lightState = LightState.OFF;
//                currentTarget = intakeSlide.targetPositionPickup2+400;
//                intakeSlide.runToPosition(currentTarget);
//                while (sensorRange.getDistance(DistanceUnit.CM) < 15) {
//                    driveStraight(0.3);
//                }
//                unloaded = false;
//                break;
//        }
//    }
//
//    public void pickUpCone() {
//        while (unloaded) {
//            pickupCone();
//        }
//    }

    // change speed (direction) for strafe right/left for different starting positions
    public void dropOffCone(double speed) {
        strafe(speed);
        while (loaded) {
            dropOff();
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
    }

    private void dropOff() {
        if (sensorRange.getDistance(DistanceUnit.CM) < 50) {
            switch (dropOffState) {
                case ALIGNING:
                    lightState = LightState.ALIGNING;
                    while (sensorRange.getDistance(DistanceUnit.CM) > 13) {
                        driveStraight(0.3);
                    }
                    if (sensorRange.getDistance(DistanceUnit.CM) <= 13) {
                        dropOffState = DropOffState.ALIGNED;
                    }
                    break;
                case ALIGNED:
                    timer.reset();
                    lightState = LightState.ALIGNED;
                    intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.HIGH;
                    intakeSlide.run();
                    while (timer.seconds() < 3) {
                        // wait
                    }
                    while (sensorRange.getDistance(DistanceUnit.CM) > 13) {
                        driveStraight(-0.3);
                    }
                    timer.reset();
                    intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.IN);
                    while (timer.seconds() < 2) {
                        intakeSlide.runIntake();
                    }
                    dropOffState = DropOffState.UNLOADED;
                    break;
                case UNLOADED:
                    intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.STOP);
                    intakeSlide.runIntake();
                    while (sensorRange.getDistance(DistanceUnit.CM) < 11) {
                        driveStraight(-0.3);
                    }
                    lightState = lightState.ALIGNING;
                    intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
                    loaded = false;
                    break;
            }
        }
    }

    // Negative speed for backwards
    private void driveStraight(double speed) {

        double leftYControl = 1*speed;
        double leftXControl = 0;
        double rightXControl = 0;

        drive.setWeightedDrivePower(
                new Pose2d(
                        -leftYControl,
                        -leftXControl,
                        -rightXControl
                )
        );
        drive.update();


    }

    // Negative speed for left
    private void strafe(double speed) {

        double leftYControl = 0;
        double leftXControl = speed;
        double rightXControl = 0;

        while (sensorRange.getDistance(DistanceUnit.CM) < 50) {
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

}
