package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
        UNALIGNED,
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
    private boolean loaded = true;
    
    // Alignment
    private int s = 0;
    private int checks = 0;
    private int positives = 0;
    private int pIterations = 0;
    private boolean alignedL = false;
    private double lastDistance;
    
    // Constants
    private final int minimumPositives = 8;

    private LinearOpMode op;

    // Hardware
    private DistanceSensor sensorRange;
//    private TouchSensor sensorTouch1;
//    private TouchSensor sensorTouch2;
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

//        sensorTouch1 = hardwareMap.get(TouchSensor.class, "sensor_touch_right");
//        sensorTouch2 = hardwareMap.get(TouchSensor.class, "sensor_touch_left");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        pickupState = PickupState.ALIGNING;
        dropOffState = DropOffState.UNALIGNED;
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
    // PROBLEM: YOU HAVE TO CALL RUN FOR INTAKE SLIDES AND INIT GAMEPADS!! HOW TO SOLVE!?
    public void pickupCone() {
        switch (pickupState) {
            case ALIGNING:
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                intakeSlide.run();
                while (sensorRange.getDistance(DistanceUnit.CM) > 9) {
                    straight(0.2);
                }
                // vvv change condition to include touch sensor vvv
                if (8 < sensorRange.getDistance(DistanceUnit.CM) && sensorRange.getDistance(DistanceUnit.CM) > 10) {
                    pickupState = PickupState.ALIGNED;
                }
                break;
            case ALIGNED:
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
                intakeSlide.run();
                while (intakeSlide.getSlidePosition() < intakeSlide.currentTarget) {
                    intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.IN);
                    intakeSlide.runIntake();
                }
                pickupState = PickupState.LOADED;
                break;
            case LOADED:
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                intakeSlide.run();
                while (sensorRange.getDistance(DistanceUnit.CM) < 15) {
                    straight(-0.3);
                }
                loaded = true;
                break;
        }
    }

    public void pickUpCone() {
        while (!loaded) {
            intakeSlide.stack = true;
            pickupCone();
        }
    }

    // change speed (direction) for strafe right/left for different starting positions
    public void dropOffCone(LinearOpMode p_op, double speed, IntakeSlideSubsystemAuto.LiftState height) {
        op = p_op;
        if (op.opModeIsActive()) {
            timer.reset();
            while (s < 2) {
                while ((sensorRange.getDistance(DistanceUnit.CM) > 40 && timer.seconds() < 5) && op.opModeIsActive() ) {
                    strafe(speed);
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State:", dropOffState.name());
                    op.telemetry.update();
                    lastDistance = sensorRange.getDistance(DistanceUnit.CM);
                }
                if (sensorRange.getDistance(DistanceUnit.CM) < lastDistance+5) {
                    s++;
                }
            }
            dropOffState = DropOffState.ALIGNING;
            s = 0;
            // troubleshooting
            while (loaded && op.opModeIsActive()) {
                dropOff(height);
            }
            stopMovement();
        }
    }

    private void dropOff(IntakeSlideSubsystemAuto.LiftState height) {
        switch (dropOffState) {
            case UNALIGNED:
                break;
            case ALIGNING:
                intakeSlide.liftState = height;
                intakeSlide.run();
                straightDistance(0.2, -2);
                timer.reset();
                while (sensorRange.getDistance(DistanceUnit.CM) < 800 && sensorRange.getDistance(DistanceUnit.CM) > 8 && timer.seconds() < 2 && op.opModeIsActive()) {
                    straight(0.1);
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State:", dropOffState.name());
                    op.telemetry.update();
                }
                if (sensorRange.getDistance(DistanceUnit.CM) < 8 || sensorRange.getDistance(DistanceUnit.CM) > 40) {
                    dropOffState = DropOffState.ALIGNED;
                }
                break;
            case ALIGNED:
                timer.reset();
                while (timer.seconds() < 2 && op.opModeIsActive()) {
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State:", dropOffState.name());
                    op.telemetry.update();
                    stopMovement();
                }
                timer.reset();
                while (timer.seconds() < 2 && op.opModeIsActive()) {
                    intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.OUT);
                    intakeSlide.runIntake();
                }
                dropOffState = DropOffState.UNLOADED;
                break;
            case UNLOADED:
                loaded = false;
                intakeSlide.setIntakePower(IntakeSlideSubsystemAuto.IntakeState.STOP);
                intakeSlide.runIntake();
                timer.reset();
                while (timer.seconds() < 1 && op.opModeIsActive()) {
                    straight(-0.3);
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.update();
                }
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                intakeSlide.run();
                op.telemetry.addData("Drop Off:", "Completed");
                op.telemetry.update();
                break;
        }
    }

    public boolean checkForAlign() {
        while (checks<=10) {
            if (sensorRange.getDistance(DistanceUnit.CM) < 40) {
                positives++;
            }
            checks++;
        }
        // if positives is greater than the minimum positives needed to be aligned
        if (positives > minimumPositives) {
            return true;
        } else {
            return false;
        }
    }
    public void align() {
        while (!checkForAlign()) {
            strafeDistance(-0.3, -1);
        }
        pIterations++;
        while (checkForAlign()) {
            strafeDistance(-0.3, -1);
            pIterations++;
        }
        for (s = 0; s < pIterations; s++) {
            strafeDistance(0.3, 1);
        }
    }

    // only stops overshoot
    public void smallAlign() {
        straightDistance(-0.3, -2);
        if (sensorRange.getDistance(DistanceUnit.CM) < 8) {
            stopMovement();
        }
    }
    
    // Negative speed for forwards
    private void straight(double speed) {

        double leftYControl = speed;
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

        drive.setWeightedDrivePower(
                new Pose2d(
                        -leftYControl,
                        -leftXControl,
                        -rightXControl
                )
        );
        drive.update();

    }

    private void stopMovement() {

        double leftYControl = 0;
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

    private void strafeDistance (double speed, double distance) {

        double leftYControl = 0 ;
        double leftXControl = Math.abs(speed);
        double rightXControl = 0;

        if (distance > 0 ){
            leftXControl = leftXControl * 1;
        } else {
            leftXControl = leftXControl * -1;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 16;

        while (controlTimer.seconds() < timeLimit){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl  ,
                            -leftXControl  ,
                            -rightXControl
                    )
            );
            drive.update();
        }

    }

    private void straightDistance (double speed, double distance) {

        double leftYControl = Math.abs(speed);
        double leftXControl = 0;
        double rightXControl = 0;

        if (distance > 0 ){
            leftYControl = leftYControl * 1;
        } else {
            leftYControl = leftYControl * -1;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 16;

        while (controlTimer.seconds() < timeLimit){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl  ,
                            -leftXControl  ,
                            -rightXControl
                    )
            );
            drive.update();
        }

    }

}
