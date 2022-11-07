package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The IntakeSlideSubsystem class is a subsystem module that control
 * the elevator and roller intake used in PowerPlay
 *
 * @author  MY, AN, SL
 * @version 1.0
 * @since   2022-10-20
 */
public class IntakeSlideSubsystem extends IntakeSlide {

    // THESE VARIABLES ARE USED BY THIS IMPLEMENTATION OF DRIVE CONTROL
    // Declare OpMode members.
    private ElapsedTime levelTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    // Variable to detect on press and on release
    private int i = 0;
    private boolean pressedLastIterationRT = false;
    private boolean pressedLastIterationRB = false;
    private boolean pressedLastIterationLB = false;
    private boolean pressedLastIterationDU = false;

    // Variable to auto spin in intake
    private boolean autoIn = false;

    /**
     *
     * @param hardwareMap from teleop
     */
    @Override
    public void init(HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intake = hardwareMap.get(CRServo.class, "intake");


        currentPosition = slides.getCurrentPosition();
        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentTarget = 0;
        currentPower = 0;

        intakeState = IntakeState.STOP;
        liftState = LiftState.REST;


    }

    public int getDpadPressed() { return i; }

    public Boolean getLastIterationRB() { return pressedLastIterationRB; }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2){
        switch (liftState) {
            case REST:
                // stops intake when slides hit rest
                if (slides.getCurrentPosition() == currentTarget && gamepad2.left_trigger == 0) {
                    autoIn = false;
                }
                if (onRelease(gamepad1.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else if (onPress(gamepad1.right_trigger > 0, "RT")) {
                    // code here
                    autoIn = false;
                    currentTarget = targetPositionPickup;
                    liftState = LiftState.PICKUP;
                } else if (onPress(gamepad1.left_bumper, "LB")) {
                    autoIn = false;
                } else if (onRelease(gamepad1.dpad_up, "DU")) {
                    i++;
                }
                setSlidePower();
                //if (gamepad1.x || gamepad1.y) {
                //   liftState = LiftState.MANUAL;
                //}
                break;
            case PICKUP:
                if (onRelease(gamepad1.right_trigger > 0, "RT")) {
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    autoIn = true;
                } else if (onRelease(gamepad1.right_bumper, "RB")) {
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else if (onRelease(gamepad1.left_bumper, "LB")) {
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    autoIn = false;
                } else if (slides.getCurrentPosition() < targetPositionLow) {
                    // add position to pick up from stack all the way to LOW
                    currentTarget += 2;
                } else {
                    setSlidePower();
                }
                break;
            case LOW:
                if (onRelease(gamepad1.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                } else if (onPress(gamepad1.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                } else {
                    setSlidePower();
                }
                //if (gamepad1.x || gamepad1.y) {
                //  liftState = LiftState.MANUAL;
                //}
                break;
            case MEDIUM:
                if (onRelease(gamepad1.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                } else if (onPress(gamepad1.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                } else {
                    setSlidePower();
                }
                //if (gamepad1.x || gamepad1.y) {
                //   liftState = LiftState.MANUAL;
                //}
                break;
            case HIGH:
                // DO SOMETHING TO MAKE DRIVING SLOWER WHILE THE CASE IS HIGH FOR BETTER CONTROL
                if (onRelease(gamepad1.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                } else {
                    setSlidePower();
                }
                //if (gamepad1.x || gamepad1.y) {
                //   liftState = LiftState.MANUAL;
                //}
                break;

                /*
            case MANUAL:
                // Backup Controls

                // Viper Slides
                if (gamepad1.y && !gamepad1.x) {
                    currentTarget += 1;
                } else if (gamepad1.x && !gamepad1.y) {
                    currentTarget -= 1;
                }

                if ((onPress(gamepad1.right_bumper, "RB")) && (currentPosition < 300)) {
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                    setSlidePower();
                } else if ((onPress(gamepad1.right_bumper, "RB")) && (currentPosition >= 300 && currentPosition < 350)) {
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                    setSlidePower();
                } else if ((onPress(gamepad1.right_bumper, "RB")) && (currentPosition >= 350 && currentPosition < 2500)) {
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                    setSlidePower();
                }
                if (onPress(gamepad1.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                }
                break;
                 */
        }
        pressedLastIterationDU = gamepad1.dpad_up;
        pressedLastIterationRB = gamepad1.right_bumper;
        pressedLastIterationLB = gamepad1.left_bumper;
        pressedLastIterationRT = gamepad1.right_trigger > 0;
        runToPosition(currentTarget, currentPower);
        runIntake(gamepad1);
        //runIntake();
    }


    /**
     * start or stop intake, depending upon the current state
     *
     *  steady state and transient state
     *  https://resources.pcb.cadence.com/blog/2020-steady-state-vs-transient-state-in-system-design-and-stability-analysis
     */
    @Override
    public void runIntake(Gamepad controller){

        switch (intakeState) {
            case STOP:
                intake.setPower(0);
                if ((controller.left_trigger > 0) || controller.b) {
                    intakeState = IntakeState.OUT;
                }
                if (controller.a || autoIn) {
                    intakeState = IntakeState.IN;
                }
                break;
            case IN:
                if (!controller.a && !autoIn) {
                    intakeState = IntakeState.STOP;
                } else if (controller.left_trigger > 0 || controller.b) {
                    autoIn = false;
                    intakeState = IntakeState.STOP;
                }
                intake.setPower(-1);
                break;
            case OUT:
                if (controller.left_trigger == 0 || !controller.b) {
                    intakeState = IntakeState.STOP;
                }
                intake.setPower(1);
                break;
        }
    }



    private boolean  onPress(boolean ButtonState, String ButtonName) {
        switch (ButtonName) {
            case "RT":
                /**
                 *  ButtonState = true when right trigger is pressed i.e. trigger > 0
                 *  ButtonState = false when trigger is not pressed i.e. trigger = 0
                 */
                if (ButtonState && !pressedLastIterationRT) {
                    return true;
                }
                break;
            case "RB":
                if (ButtonState && !pressedLastIterationRB) {
                    return true;
                }
                break;
            case "LB":
                if (ButtonState && !pressedLastIterationLB) {
                    return true;
                }
                break;
            case "DU":
                if (ButtonState && !pressedLastIterationDU) {
                    return true;
                }
                break;
        }
        return false;
    }
    private boolean onRelease(boolean ButtonState, String ButtonName) {
        switch (ButtonName) {
            case "RT":
                if (!ButtonState && pressedLastIterationRT) {
                    return true;
                }
                break;
            case "RB":
                if (!ButtonState && pressedLastIterationRB) {
                    return true;
                }
                break;
            case "LB":
                if (!ButtonState && pressedLastIterationLB) {
                    return true;
                }
                break;
            case "DU":
                if (!ButtonState && pressedLastIterationDU) {
                    return true;
                }
                break;
        }
        return false;
    }
}