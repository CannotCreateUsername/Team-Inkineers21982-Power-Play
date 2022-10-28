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
public class IntakeSlideSubsystem2 {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime levelTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private DcMotor slides = null;
    private CRServo intake = null;

    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
    private final int targetPositionHigh = 2700;
    private final int targetPositionMedium = 350;
    private final int targetPositionLow = 300;
    private final int targetPositionPikcup = 250;
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

    private double currentPower;
    private int currentTarget;
    private String currentCaption;
    private String currentStatus;
    private String currentLevel;

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
    public IntakeSlideSubsystem2(HardwareMap hardwareMap) {

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


        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentLevel = "Rest";
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

    public String getCurrentCaption(){
        return currentCaption;
    }

    public String getCurrentStatus(){
        return currentStatus;
    }



    /**
     *   set the power and status
     */
    private void setSlidePower(double power){
        if (Math.abs(slides.getCurrentPosition()- currentTarget) > 15){
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            currentPower = power;
            if (currentTarget > targetPositionRest+20 && currentTarget < targetPositionLow-20) {
                currentLevel = "Pickup";
            } else if (currentTarget > targetPositionPikcup+20 && currentTarget < targetPositionMedium-20) {
                currentLevel = "Low";
            } else if (currentTarget > targetPositionLow+20 && currentTarget < targetPositionHigh-20) {
                currentLevel = "Medium";
            } else if (currentTarget > targetPositionMedium+20) {
                currentLevel = "High";
            } else {
                currentLevel = "Rest";
            }
            currentStatus = "Going to: " + currentTarget + currentLevel;
        } else {
            double posErr = currentTarget - slides.getCurrentPosition(); // measure error in terms of distance between current position and target
            currentPower = (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            currentStatus = "Holding at: " + slides.getCurrentPosition() + currentLevel;
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
                if (gamepad1.y) {
                    levelTimer.reset();
                    timesPressed += 1;
                    while (levelTimer.time() < 2) {
                        // Within 2 seconds:
                        if (gamepad1.y) {
                            timesPressed += 1;
                        }
                    }
                    if (timesPressed == 1) {
                        // If pressed one time, go low
                        currentTarget = targetPositionLow;
                        liftState = LiftState.LOW;
                    } else if (timesPressed == 2) {
                        // If pressed two times, go middle
                        currentTarget = targetPositionMedium;
                        liftState = liftState.MEDIUM;
                    } else if (timesPressed > 2) {
                        // If pressed three or more times, go to high
                        currentTarget = targetPositionHigh;
                        liftState = LiftState.HIGH;
                    }
                    // Reset timesPressed
                    timesPressed = 0;
                }
                break;
            case PICKUP:
                if (gamepad1.left_bumper) {
                    // left_trigger is pressed, go to rest position and turn on the intake
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;

                    // start intake as well
                    intakeState = IntakeState.IN;
                    intakeTimer.reset();

                    // use slow power when it get down
                    setSlidePower(0.09);
                } else {
                    setSlidePower();
                }
                break;
            case HIGH:
                if (gamepad1.x) {
                    // from high to rest state
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                } else if (gamepad1.right_bumper) {
                    // release intake
                    intakeState = IntakeState.OUT;
                    intakeTimer.reset();
                } else {
                    setSlidePower();
                }
                break;
            case MEDIUM:
                if (gamepad1.x) {
                    // from high to rest state
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                } else if (gamepad1.right_bumper) {
                    // release intake
                    intakeState = IntakeState.OUT;
                    intakeTimer.reset();
                } else {
                    setSlidePower();
                }
                break;
            case LOW:
                if (gamepad1.x) {
                    // from high to rest state
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                } else if (gamepad1.right_bumper) {
                    // release intake
                    intakeState = IntakeState.OUT;
                    intakeTimer.reset();
                } else {
                    setSlidePower();
                }
                break;
        }

        runToPosition(currentTarget, currentPower);
        runIntake();


    }


    /**
     * start or stop intake, depending upon the current state
     *
     *  steady state and transient state
     *  https://resources.pcb.cadence.com/blog/2020-steady-state-vs-transient-state-in-system-design-and-stability-analysis

     */
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

}
