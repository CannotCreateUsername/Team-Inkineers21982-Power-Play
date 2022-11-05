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
public class IntakeSlideSubsystem {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime levelTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    // Variable to detect on press and on release
    private boolean pressedLastIterationRT = false;
    private boolean pressedLastIterationRB = false;
    private boolean pressedLastIterationLB = false;

    // Variable to auto spin in intake
    private boolean autoIn = false;

    private DcMotor slides = null;
    private CRServo intake = null;

    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
    private final int targetPositionHigh = 2500;
    private final int targetPositionMedium = 1200;
    private final int targetPositionLow = 400;
    private final int targetPositionPickup = 130;
    private final int targetPositionRest = 0;  // ideally it should be zero !!!

    // distance error factor
    // https://gm0.org/en/latest/docs/software/concepts/control-loops.html?highlight=pid#built-in-pid-controller
    // 2022-10-19: THIS NUMBER MIGHT NEED TO BE TUNED !!!
    private final double Kp = .05;

    // 2022-10-19: THE DEFAULT POWER MIGHT NEED TO BE BE TUNED !!!
    private double defaultPower = 0.7;
    private double defalutVelocity = 200;
    private double defaultIntakeTime = 2.0;

    private int timesPressed;
    private boolean triggerPressed;
    
    private int currentPosition;
    private double currentPower;
    private int currentTarget;
    private String currentCaption;
    private String currentStatus;

    private LiftState liftState;
    private IntakeState intakeState;

    private boolean intakeToggle;

    // 2022-10-19: REVIEW THE STATE !!!
    public enum LiftState {
        REST,   // all the way to the bottom
        PICKUP,  // level where the robot will pickup the cone
        LOW,
        MEDIUM,
        HIGH,
        MANUAL
    }

    public enum IntakeState {
        STOP,
        IN,
        OUT
    }

    /**
     *
     * @param hardwareMap from teleop
     */
    public IntakeSlideSubsystem(HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intake = hardwareMap.get(CRServo.class, "intake");

        runtime.reset();

        currentPosition = slides.getCurrentPosition();
        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentTarget = 0;
        currentPower = 0;

        intakeState = IntakeState.STOP;
        liftState = LiftState.REST;


        intakeToggle = false;


    }

    /**
     *
     * @param position where the slider will travel to. It is measured in motor tickts
     * @param power the power of motor ,
     */
    private void runToPosition(int position, double power){
        slides.setTargetPosition(position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TO DO: ACCORDING TO THIS LINK, TYPICALLY WE SET THE MAX VELOCITY INSTEAD OF POWER WHEN USING RUN_TO_POSITION
        https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback
        slides.setPower(power);
    }

    private void runToPosition(int position){

        runToPosition (position, defaultPower);
    }

    public String getCurrentCaption() {
        return currentCaption;
    }

    public String getCurrentStatus() {
        return currentStatus;
    }
    
    public int getCurrentSlidePosition() {
        return currentPosition;
    }

    /**
     *   set the power and status
     */
    private void setSlidePower(double power){
        if (Math.abs(currentPosition- currentTarget) > 15){
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            currentPower = power;
            currentStatus = "Going to: " + currentTarget;
        } else {
            double posErr = currentTarget - currentPosition; // measure error in terms of distance between current position and target
            currentPower = (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            currentStatus = "Holding at: " + currentPosition;
        }
    }

    /**
     *  set slide power and status with default Power
     */
    private void setSlidePower(){
        setSlidePower(defaultPower);
    }


    /**
     https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
     button mapping
     gamepad1.y = high
     gamepad1.x = medium
     gamepad1.b = low
     gamepad1.a = pickup
     gamepad1.left_bumper = rest
     gamepad1.right_bumper= release
     */
    public void run(Gamepad gamepad1, Gamepad gamepad2){
        switch (liftState) {
            case REST:
                // stops intake when slides hit rest
                if (currentPosition == currentTarget) {
                    autoIn = false;
                }
                if (onPress(gamepad2.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else if (onPress(gamepad2.right_trigger > 0, "RT")) {
                    // code here
                    autoIn = false;
                    currentTarget = targetPositionPickup;
                    liftState = LiftState.PICKUP;
                } else if (onPress(gamepad2.left_bumper, "LB")) {
                    autoIn = false;
                }
                // Manual
                if (gamepad2.x || gamepad2.y || gamepad2.a || gamepad2.b) {
                    liftState = LiftState.MANUAL;
                }
                /*
                if (gamepad1.y) {
                    // y is pressed to to High postion
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                } else if (gamepad1.x) {
                    // x is pressed, go to medium position
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                } else if (gamepad1.b) {
                    // b is pressed, go to low position
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else if (gamepad1.a) {
                    // a is pressed, go to pickup position
                    currentTarget = targetPositionPickup;
                    liftState = LiftState.PICKUP;
                }
                 */
                break;
            case PICKUP:
                if (onRelease(gamepad2.right_trigger > 0, "RT")) {
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    autoIn = true;
                    setSlidePower(0.09);
                } else if (onPress(gamepad2.right_bumper, "RB")) {
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                    setSlidePower();
                } else if (onPress(gamepad2.left_bumper, "LB")) {
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    autoIn = false;
                    setSlidePower();
                } else if (currentPosition < targetPositionLow) {
                    // add position to pick up from stack all the way to LOW
                    currentTarget += 3;
                    setSlidePower();
                } else {
                    setSlidePower();
                }
                break;
            case LOW:
                if (onPress(gamepad2.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                    setSlidePower();
                }
                if (onPress(gamepad2.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                }
                if (gamepad2.x || gamepad2.y) {
                    liftState = LiftState.MANUAL;
                }
            case MEDIUM:
                if (onPress(gamepad2.right_bumper, "RB")) {
                    // code here
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                    setSlidePower();
                }
                if (onPress(gamepad2.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                }
                if (gamepad2.x || gamepad2.y) {
                    liftState = LiftState.MANUAL;
                }
            case HIGH:
                // DO SOMETHING TO MAKE DRIVING SLOWER WHILE THE CASE IS HIGH FOR BETTER CONTROL
                if (onPress(gamepad2.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                }
                if (gamepad2.x || gamepad2.y) {
                    liftState = LiftState.MANUAL;
                }
                break;
            case MANUAL:
                // Backup Controls

                // Viper Slides
                if (gamepad2.y && !gamepad2.x) {
                    currentTarget += 1;
                } else if (gamepad2.x && !gamepad2.y) {
                    currentTarget -= 1;
                }

                if ((onPress(gamepad2.right_bumper, "RB")) && (currentPosition < 300)) {
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                    setSlidePower();
                } else if ((onPress(gamepad2.right_bumper, "RB")) && (currentPosition >= 300 && currentPosition < 350)) {
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                    setSlidePower();
                } else if ((onPress(gamepad2.right_bumper, "RB")) && (currentPosition >= 350 && currentPosition < 2500)) {
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                    setSlidePower();
                }
                if (onPress(gamepad2.left_bumper, "LB")) {
                    // code here
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                }
                break;
        }

        runToPosition(currentTarget, currentPower);
        runIntake2(gamepad2);
        //runIntake();
    }


    /**
     * start or stop intake, depending upon the current state
     *
     *  steady state and transient state
     *  https://resources.pcb.cadence.com/blog/2020-steady-state-vs-transient-state-in-system-design-and-stability-analysis
     */
    /*
    private void runIntake(){
        switch (intakeState) {
            case STOP:
                intake.setPower(0);
                break;
            case IN:
                if (intakeTimer.seconds() <= defaultIntakeTime) {
                    intake.setPower(-1);
                } else {
                    intakeState = IntakeState.STOP;
                }
                break;
            case OUT:
                if (intakeTimer.seconds() <= defaultIntakeTime) {
                    intake.setPower(1);
                } else {
                    intakeState = IntakeState.STOP;
                }
                break;
        }
    }
     */
    private void runIntake2(Gamepad controller){

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
        }
    }


    private boolean onPress(boolean ButtonState, String ButtonName) {
        switch (ButtonName) {
            case "RT":
                if (ButtonState && !pressedLastIterationRT) {
                    return true;
                }
                pressedLastIterationRT = ButtonState;
            case "RB":
                if (ButtonState && !pressedLastIterationRB) {
                    return true;
                }
                pressedLastIterationRB = ButtonState;
            case "LB":
                if (ButtonState && !pressedLastIterationLB) {
                    return true;
                }
                pressedLastIterationLB = ButtonState;
        }
        return false;
    }
    private boolean onRelease(boolean ButtonState, String ButtonName) {
        switch (ButtonName) {
            case "RT":
                if (!ButtonState && pressedLastIterationRT) {
                    return true;
                }
                pressedLastIterationRT = ButtonState;
        }
        return false;
    }
}