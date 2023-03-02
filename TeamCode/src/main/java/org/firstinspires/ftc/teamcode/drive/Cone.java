package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cv.StickDriveMediator;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlideSubsystemAuto;

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

    ElapsedTime timer = new ElapsedTime();
    public boolean loaded = true;
    
    // Alignment
    private int s = 0;
    private int checks = 0;
    private int positives = 0;
    private int pIterations = 0;
    private boolean alignedL = false;
    private double lastDistance;

    public boolean drop = false;
    
    // Constants
    private final int MINIMUM_POSITIVES = 8;

    private final double JUNCTION_DISTANCE = 8;
    private final double CONE_DISTANCE = 6;
    private final double LATERAL_DISTANCE = 40;

    // Hardware
    private DistanceSensor sensorRange;
    private TouchSensor sensorTouch1;
    private TouchSensor sensorTouch2;

    PickupState pickupState;
    DropOffState dropOffState;

    IntakeSlideSubsystemAuto intakeSlide;
    SampleMecanumDrive drive;
    LinearOpMode op;
    public StickDriveMediator stickDrive;

    public void init (SampleMecanumDrive d, IntakeSlideSubsystemAuto i, HardwareMap hardwareMap, LinearOpMode o) {
        drive = d;
        intakeSlide = i;
        op = o;
        stickDrive = new StickDriveMediator(op);
        stickDrive.setDrive(drive);
        stickDrive.setSlide(intakeSlide);
        // stickDrive.observeStick();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorTouch1 = hardwareMap.get(TouchSensor.class, "leftTouch");
        sensorTouch2 = hardwareMap.get(TouchSensor.class, "rightTouch");

        pickupState = PickupState.ALIGNING;
        dropOffState = DropOffState.UNALIGNED;

    }

//    private boolean againstWall() {
//        if (sensorTouch1.isPressed() && sensorTouch2.isPressed()) {
//            return true;
//        } else {
//            return false;
//        }
//    }
    private void pickCone() {
        switch (pickupState) {
            case ALIGNING:
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                intakeSlide.run();
                timer.reset();
                //|| !againstWall()
                //|| (sensorTouch1.isPressed() && sensorTouch2.isPressed())
                while ((sensorRange.getDistance(DistanceUnit.CM) > CONE_DISTANCE || (sensorTouch1.isPressed() && sensorTouch2.isPressed())) && timer.seconds() < 5 && op.opModeIsActive()) {
                    straight(0.3);
                    op.telemetry.addData("State", pickupState.name());
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("Lift State", intakeSlide.getCurrentState());
                    op.telemetry.update();
                }
                stopMovement();
                // vvv change condition to include touch sensor vvv
                // touch sensor 1 || 2 is pressed
                if (8 > sensorRange.getDistance(DistanceUnit.CM) || sensorTouch1.isPressed() || sensorTouch2.isPressed() || timer.seconds() < 6) {
                    pickupState = PickupState.ALIGNED;
                }

                break;
            case ALIGNED:
                timer.reset();
                intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.IN);
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.REST;
                intakeSlide.run();
                while (timer.seconds() < 0.5) {
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State", pickupState.name());
                    op.telemetry.addData("Lift State", intakeSlide.getCurrentState());
                    op.telemetry.update();
                }
                pickupState = PickupState.LOADED;
                break;
            case LOADED:
                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.LOW;
                intakeSlide.run();
                timer.reset();
                while (timer.seconds() < 1 && op.opModeIsActive()) {
                    // wait
                }
                // back out
                while (sensorRange.getDistance(DistanceUnit.CM) < 18 && timer.seconds() < 3 && op.opModeIsActive()) {
                    straight(-0.3);
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State", pickupState.name());
                    op.telemetry.addData("Lift State", intakeSlide.getCurrentState());
                    op.telemetry.update();
                }
                loaded = true;
                break;
        }
    }

    public void pickUpCone() {
        loaded = false;
        while (!loaded && op.opModeIsActive()) {
            intakeSlide.stack = true;
            pickCone();
        }
    }

    // change speed (direction) for strafe right/left for different starting positions
    public void dropOffCone(double speed, IntakeSlideSubsystemAuto.LiftState height, boolean cone) {
        if (op.opModeIsActive()) {
            timer.reset();
            while ((sensorRange.getDistance(DistanceUnit.CM) > LATERAL_DISTANCE && timer.seconds() < 5) && op.opModeIsActive() ) {
                strafe(speed);
                op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                op.telemetry.update();
                lastDistance = sensorRange.getDistance(DistanceUnit.CM);
            }
            dropOffState = DropOffState.ALIGNING;
            s = 0;
            // troubleshooting
            while (loaded && op.opModeIsActive()) {
                dropOff(height, cone);
                if (!loaded) {
                    stopMovement();
                }
            }
            // cone stack height changes if there is less cones
            intakeSlide.stackDiff = intakeSlide.stackDiff - 100;
        }
    }

    private void dropOff(IntakeSlideSubsystemAuto.LiftState height, boolean cone) {
        switch (dropOffState) {
            case UNALIGNED:
                break;
            case ALIGNING:
                intakeSlide.liftState = height;
                intakeSlide.run();
                timer.reset();

                // whether a cone has already been dropped
                if (!cone && sensorRange.getDistance(DistanceUnit.CM) < LATERAL_DISTANCE) {
                    while ((sensorRange.getDistance(DistanceUnit.CM) < 400 && sensorRange.getDistance(DistanceUnit.CM) > JUNCTION_DISTANCE) && timer.seconds() < 3 && op.opModeIsActive()) {
                        straight(0.1);
                        op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                        op.telemetry.addData("State:", dropOffState.name());
                        op.telemetry.update();
                    }
                } else if (cone && sensorRange.getDistance(DistanceUnit.CM) < LATERAL_DISTANCE) {
                    while ((sensorRange.getDistance(DistanceUnit.CM) < 400 && sensorRange.getDistance(DistanceUnit.CM) > CONE_DISTANCE) && timer.seconds() < 3 && op.opModeIsActive()) {
                        straight(0.1);
                        op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                        op.telemetry.addData("State:", dropOffState.name());
                        op.telemetry.update();
                    }
                } else {
                    straightDistance(3);
                }

                if (sensorRange.getDistance(DistanceUnit.CM) < 15 || sensorRange.getDistance(DistanceUnit.CM) > LATERAL_DISTANCE) {
                    dropOffState = DropOffState.ALIGNED;
                }
                break;
            case ALIGNED:
                timer.reset();
                while (timer.seconds() < 0.2) {
                    op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                    op.telemetry.addData("State:", dropOffState.name());
                    op.telemetry.update();
                    stopMovement();
                }

                timer.reset();
                while (timer.seconds() < 1.5 && op.opModeIsActive()) {
                    intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.OUT);
                }
                dropOffState = DropOffState.UNLOADED;
                break;
            case UNLOADED:
                intakeSlide.runIntake();
                timer.reset();

//                if (sensorRange.getDistance(DistanceUnit.CM) < LATERAL_DISTANCE) {
//                    while (sensorRange.getDistance(DistanceUnit.CM) < 18 && timer.seconds() < 2 && op.opModeIsActive()) {
//                        straight(-0.3);
//                        op.telemetry.addData("Time", timer.seconds());
//                        op.telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
//                        op.telemetry.update();
//                    }
//                } else {
//                    straightDistance(-8);
//                }

                straightDistance(-8);

                loaded = false;

                intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                intakeSlide.run();
                op.telemetry.addData("Drop Off:", "Completed");
                op.telemetry.update();
                dropOffState = DropOffState.UNALIGNED;
                break;
        }
    }

//    public boolean checkForAlign() {
//        while (checks<=10) {
//            if (sensorRange.getDistance(DistanceUnit.CM) < LATERAL_DISTANCE) {
//                positives++;
//            }
//            checks++;
//        }
//        // if positives is greater than the minimum positives needed to be aligned
//        if (positives > MINIMUM_POSITIVES) {
//            return true;
//        } else {
//            return false;
//        }
//    }
//    public void align() {
//        while (!checkForAlign()) {
//            strafeDistance(-1);
//        }
//        pIterations++;
//        while (checkForAlign()) {
//            strafeDistance(-1);
//            pIterations++;
//        }
//        for (s = 0; s < pIterations/2; s++) {
//            strafeDistance(1);
//        }
//    }
//
//    public void smallAlignL() {
//        strafeDistance2(20);
//    }
//
    // only stops overshoot
    public void smallAlignV(boolean cone) {
        straightDistance(8);
        if (!cone && sensorRange.getDistance(DistanceUnit.CM) < JUNCTION_DISTANCE) {
            stopMovement();
        } else if (cone && sensorRange.getDistance(DistanceUnit.CM) < CONE_DISTANCE) {
            stopMovement();
        }
    }

    public void align(IntakeSlideSubsystemAuto.LiftState height, boolean coneThere) {
        if (drop) {
            stickDrive.alignStick(6,3, height, coneThere);
            timer.reset();
            while (timer.seconds() < 1.5 && op.opModeIsActive()) {
                intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.OUT);
            }
            straightDistance(-10);
            intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
            intakeSlide.run();
            // stop camera stream to conserve CPU
            stickDrive.stopCamera();
            op.telemetry.addData("Drop Off:", "Completed");
            op.telemetry.update();
            loaded = false;
            drop = false;
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

    private void strafeDistance (double distance) {

        double leftYControl = 0 ;
        double leftXControl;
        double rightXControl = 0;

        if (distance > 0 ){
            leftXControl = 0.3;
        } else {
            leftXControl = -0.3;
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

    private void straightDistance (double distance) {

        double leftYControl;
        double leftXControl = 0;
        double rightXControl = 0;

        if (distance > 0 ){
            leftYControl = 0.3;
        } else {
            leftYControl = -0.3;
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

    private void strafeDistance2 (double distance) {

        double leftYControl = 0 ;
        double leftXControl;
        double rightXControl = 0;

        if (distance > 0 ){
            leftXControl = 0.3;
        } else {
            leftXControl = -0.3;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 16;

        while (controlTimer.seconds() < timeLimit && sensorRange.getDistance(DistanceUnit.CM) > 50){
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

    // clcoksiwse = positive power
    public void turnAlign (double power) {

        double leftYControl = 0 ;
        double leftXControl = 0 ;
        double rightXControl = power;

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
