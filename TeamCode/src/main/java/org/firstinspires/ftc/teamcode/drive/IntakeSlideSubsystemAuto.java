package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSlideSubsystemAuto {
    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
    public final int targetPositionHigh = 2842;
    public final int targetPositionMedium = 2025;
    public final int targetPositionLow = 1175;
    public final int targetPositionPickup = 300;
    public final int targetPositionPickup2 = 700;
    public final int targetPositionRest = 0;  // ideally it should be zero !!!
    public final int stackDiff = 400;

    // distance error factor
    // https://gm0.org/en/latest/docs/software/concepts/control-loops.html?highlight=pid#built-in-pid-controller
    // 2022-10-19: THIS NUMBER MIGHT NEED TO BE TUNED !!!
    public final double Kp = .05;

    // 2022-10-19: THE DEFAULT POWER MIGHT NEED TO BE BE TUNED !!!
    public double defaultPower = 0.7;
    private double defalutVelocity = 200;
    public double defaultIntakeTime = 2.0;

    public int currentPosition;
    public double currentPower;
    public int currentTarget;
    public String currentCaption;
    public String currentStatus;

    public LiftState liftState;
    public IntakeState intakeState;

    public DcMotor slides = null;
    public CRServo intake = null;

    // 2022-10-19: REVIEW THE STATE !!!
    public enum LiftState {
        REST,   // all the way to the bottom
        PICKUP,  // level where the robot will pickup the cone
        PICKUP2,
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

    public boolean stack = false;


    // ***********************************************
    // Non-abstract methods
    // Having as default implementation

    /**
     *
     * @param position where the slider will travel to. It is measured in motor tickts
     * @param power the power of motor ,
     */
    public void runToPosition(int position, double power){
        slides.setTargetPosition(position);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TO DO: ACCORDING TO THIS LINK, TYPICALLY WE SET THE MAX VELOCITY INSTEAD OF POWER WHEN USING RUN_TO_POSITION
        https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback
        slides.setPower(power);
    }

    public void runToPosition(int position){

        runToPosition (position, defaultPower);
    }
    public String getCurrentState() { return liftState.name(); }

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
    public void setSlidePower(double power){
        if (Math.abs(slides.getCurrentPosition() - currentTarget) > 15){
            // our threshold is within
            // 15 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            currentPower = power;
            currentStatus = "Going to: " + currentTarget;
        } else {
            double posErr = currentTarget - slides.getCurrentPosition(); // measure error in terms of distance between current position and target
            currentPower = (posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            currentStatus = "Holding at: " + slides.getCurrentPosition();
        }
    }

    /**
     *  set slide power and status with default Power
     */
    public void setSlidePower(){
        setSlidePower(defaultPower);
    }


    public void setIntakePower(IntakeState state ){
        if (state == IntakeState.IN){
            intake.setPower(-1);
        } else if  (state == IntakeState.OUT){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

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

        intakeState = IntakeState.STOP;
        liftState = LiftState.REST;

    }

    public void run() {
        switch (liftState) {
            case REST:
                if (stack) {
                    currentTarget = targetPositionRest+stackDiff;
                } else {
                    currentTarget = targetPositionRest;
                }
                break;
            case PICKUP:
                currentTarget = targetPositionPickup;
                break;
            case PICKUP2:
                currentTarget = targetPositionPickup2;
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

    public int getSlidePosition() { return slides.getCurrentPosition(); }
    public int getCurrentTarget() { return currentTarget; }
}
