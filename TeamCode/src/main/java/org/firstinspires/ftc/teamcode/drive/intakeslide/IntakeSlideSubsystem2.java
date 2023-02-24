package org.firstinspires.ftc.teamcode.drive.intakeslide;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlide;

/**
 * The IntakeSlideSubsystem class is a subsystem module that control
 * the elevator and roller intake used in PowerPlay
 *
 * @author  MY, AN, SL
 * @version 1.0
 * @since   2022-10-20
 */
// to do: implement a virtual class for both subsystem
// https://www.geeksforgeeks.org/difference-between-abstract-class-and-interface-in-java/?ref=lbp
public class IntakeSlideSubsystem2 extends IntakeSlide {

    // THESE VARIABLES ARE USED BY THIS IMPLEMENTATION OF DRIVE CONTROL
    private ElapsedTime intakeTimer = new ElapsedTime();
    private DistanceSensor sensorRange;

//
//    // THESE ARE VARIABLES INHERID FROM ABSTRACT CLASS
//    private DcMotor slides = null;
//    private CRServo intake = null;
//
//    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
//    private final int targetPositionHigh = 2700;
//    private final int targetPositionMedium = 350;
//    private final int targetPositionLow = 300;
//    private final int targetPositionPickup = 250;
//    private final int targetPositionRest = 120;  // ideally it should be zero !!!
//
//    // distance error factor
//    // https://gm0.org/en/latest/docs/software/concepts/control-loops.html?highlight=pid#built-in-pid-controller
//    // 2022-10-19: THIS NUMBER MIGHT NEED TO BE TUNED !!!
//    private final double Kp = .05;
//
//    // 2022-10-19: THE DEFAULT POWER MIGHT NEED TO BE BE TUNED !!!
//    private double defaultPower = 0.8;
//    private double defalutVelocity = 200;
//    private double defaultIntakeTime = 2.3;
//
//
//    private double currentPower;
//    private int currentTarget;
//    private String currentCaption;
//    private String currentStatus;
//
//
//
//    // 2022-10-19: REVIEW THE STATE !!!
//    public enum LiftState {
//        REST,   // all the way to the bottom
//        PICKUP,  // level where the robot will pickup the cone
//        LOW,
//        MEDIUM,
//        HIGH,
//    }
//
//    public enum IntakeState {
//        STOP,
//        IN,
//        OUT
//    }



//    /**
//     *
//     * @param position where the slider will travel to. It is measured in motor tickts
//     * @param power the power of motor ,
//     */
//    private void runToPosition(int position, double power){
//        slides.setTargetPosition(position);
//        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // TO DO: ACCORDING TO THIS LINK, TYPICALLY WE SET THE MAX VELOCITY INSTEAD OF POWER WHEN USING RUN_TO_POSITION
//        https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback
//        slides.setPower(power);
//    }
//
//    private void runToPosition(int position){
//
//        runToPosition (position, defaultPower);
//    }
//
//    public String getCurrentCaption(){
//        return currentCaption;
//    }
//
//    public String getCurrentStatus(){
//        return currentStatus;
//    }
//
//    public String getCurrentState() {
//        return liftState.name();
//    }
//
//    public int getCurrentSlidePosition() {
//        return slides.getCurrentPosition();
//    }
//
//    /**
//     *   set the power and status
//     */
//    private void setSlidePower(double power){
//        if (Math.abs(slides.getCurrentPosition()- currentTarget) > 15){
//            // our threshold is within
//            // 15 encoder ticks of our target.
//            // this is pretty arbitrary, and would have to be
//            // tweaked for each robot.
//            currentPower = power;
//            currentStatus = "Going to: " + currentTarget;
//        } else {
//            double posErr = currentTarget - slides.getCurrentPosition(); // measure error in terms of distance between current position and target
//            currentPower = (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
//            currentStatus = "Holding at: " + slides.getCurrentPosition();
//        }
//    }
//
//    /**
//     *  set slide power and status with default Power
//     */
//    private void setSlidePower(){
//        setSlidePower(defaultPower);
//    }


    /**
     *
     * @param hardwareMap from teleop
     */
    @Override public void init(@NonNull HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Distance Sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the hardware map of intake
        intake = hardwareMap.get(CRServo.class, "intake");


        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentTarget = 0;
        currentPower = 0;

        intakeState = IntakeState.STOP;
        liftState = LiftState.REST;



    }


    /**
     https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
     button mapping
     gamepad1.y = high
     gamepad1.x = low
     gamepad1.b = medium
     gamepad1.a = pickup
     gamepad1.left_bumper = rest
     gamepad1.right_bumper= release
     * @param gamepad1
     * @param gamepad2
     */
    @Override
    public void run(GamepadEx gamepad1, GamepadEx gamepad2){
        switch (liftState) {
            case REST:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
                    // y is pressed to to High postion
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.X)) {
                    // x is pressed, go to low position
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
                    // b is pressed, go to medium position
                    currentTarget = targetPositionMedium;
                    liftState = LiftState.MEDIUM;
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                    // a is pressed, go to pickup position
                    currentTarget = targetPositionPickup;
                    liftState = LiftState.PICKUP;
                }
                // use default power to go to all positions
                setSlidePower();
                break;
            case PICKUP:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    // left_trigger is pressed, go to rest position and turn on the intake
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;

                    // start intake as well
                    intakeState = IntakeState.IN;
                    intakeTimer.reset();

                    // use slow power when it get down
                    setSlidePower(0.09);

                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    // release intake from pickup.
                    // the goal is put it in ground junction
                    // TO DO: need to test
                    intakeState = IntakeState.OUT;
                    intakeTimer.reset();

                } else {
                    // hold to current position using default power
                    setSlidePower();
                }
                break;
            case HIGH: case MEDIUM: case LOW:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    // from high to rest state
                    currentTarget = targetPositionRest;
                    liftState = LiftState.REST;
                    setSlidePower();
                } else if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    // release intake
                    intakeState = IntakeState.OUT;
                    intakeTimer.reset();
                } else {
                    // hold to the current position  using default power
                    setSlidePower();
                }
                break;
        }

        gamepad1.readButtons();
        runToPosition(currentTarget, currentPower);
        runIntake(gamepad1);


    }


    /**
     * start or stop intake, depending upon the current state
     *
     *  steady state and transient state
     *  https://resources.pcb.cadence.com/blog/2020-steady-state-vs-transient-state-in-system-design-and-stability-analysis
     */
    @Override public void runIntake(GamepadEx controller) {

        switch (intakeState) {
            case STOP:
                setIntakePower(IntakeState.STOP);   //  // intake.setPower(0);
                break;
            case IN:
                if (intakeTimer.seconds() <= defaultIntakeTime) {
                    setIntakePower(IntakeState.IN);   // intake.setPower(-1);
                } else {
                    intakeState = IntakeState.STOP;
                }
                break;
            case OUT:
                if (intakeTimer.seconds() <= defaultIntakeTime) {
                    setIntakePower(IntakeState.OUT);   // intake.setPower(1);
                } else {
                    intakeState = IntakeState.STOP;
                }
                break;
        }

    }
}