package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSlideSubystemAuto extends IntakeSlide {
    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

    }

    @Override
    public void run(GamepadEx gamepad1, GamepadEx gamepad2) {
        switch (liftState) {
            case REST:
                currentTarget = targetPositionRest;
                break;
            case PICKUP2:
                currentTarget = targetPositionPickup2 + 400;
                break;
            case LOW:
                currentTarget = targetPositionLow;
                break;
            case MEDIUM:
                currentTarget = targetPositionMedium;
                break;
            case HIGH:
                currentTarget = targetPositionHigh;
                break;
        }
        runToPosition(currentTarget);
    }

    @Override
    public void runIntake(GamepadEx controller) {
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

    public int getSlidePosition() { return slides.getCurrentPosition(); }
    public int getCurrentTarget() { return currentTarget; }
}
