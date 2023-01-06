package org.firstinspires.ftc.teamcode.drive;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Cone {

    private enum PickupState {
        ALIGNING,
        ALIGNED,
        LOADED
    }

    // Hardware
    private DistanceSensor sensorRange;
    PickupState pickupState;

    public void init(HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Distance Sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the hardware map of intake
        intake = hardwareMap.get(CRServo.class, "intake");


        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentTarget = 0;
        currentPower = 0;

        intakeState = IntakeSlide.IntakeState.STOP;
        liftState = IntakeSlide.LiftState.REST;
        pickupState = PickupState.ALIGNING;


    }

    public void run() {
        switch (liftState) {
            case REST:
                currentTarget = targetPositionRest;
                runToPosition(currentTarget);
                break;
            case PICKUP2:
                currentTarget = targetPositionPickup2 + 400;
                runToPosition(currentTarget);
                break;
            case LOW:
                currentTarget = targetPositionLow;
                runToPosition(currentTarget);
                break;
            case MEDIUM:
                currentTarget = targetPositionMedium;
                runToPosition(currentTarget);
                break;
            case HIGH:
                currentTarget = targetPositionHigh;
                runToPosition(currentTarget);
                break;
        }
    }

    public void runIntake() {
        switch (intakeState) {
            case STOP:
                setIntakePower(intakeState.STOP);
                break;
            case IN:
                setIntakePower(intakeState.IN);
                break;
            case OUT:
                setIntakePower(intakeState.OUT);
                break;
        }
    }

    public void pickupCone() {
        switch (pickupState) {
            case ALIGNING:
                liftState = LiftState.PICKUP2;
                while (sensorRange.getDistance(DistanceUnit.CM) > 10) {
                    straight(drive, 10);
                }
                if (8 < sensorRange.getDistance(DistanceUnit.CM) && sensorRange.getDistance(DistanceUnit.CM) > 10) {
                    pickupState = PickupState.ALIGNED;
                }
                break;
            case ALIGNED:
                while (slides.getCurrentPosition() < currentTarget) {
                    currentTarget = targetPositionRest+400;
                    runToPosition(currentTarget);
                    setIntakePower(intakeState.IN);
                }
                if (slides.getCurrentPosition() > currentTarget-2 && slides.getCurrentPosition() < currentTarget+2) {
                    pickupState = PickupState.LOADED;
                }
                break;
            case LOADED:

                break;
        }
    }

    private void straight (SampleMecanumDrive drive, double distance){

        double leftYControl = 0.3;
        double leftXControl = 0 ;
        double rightXControl = 0;

        while (sensorRange.getDistance(DistanceUnit.CM) > distance) {
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
