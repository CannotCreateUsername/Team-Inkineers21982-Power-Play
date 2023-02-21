package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class IntakeSlide2 {

    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
    public int targetPositionHigh = 2842;
    public int targetPositionMedium = 2025;
    public int targetPositionLow = 1175;
    public int targetPositionPickup2 = 300;
    public final int targetPositionPickup = 130;
    public final int targetPositionRest = 0;  // ideally it should be zero !!!

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
    // Regular servo
    public Servo intake = null;

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
        IN,
        OUT
    }




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
            intake.setPosition(-1);
        } else if  (state == IntakeState.OUT){
            intake.setPosition(1);
        } else {
            intake.setPosition(0);
        }
    }

    // ***********************************************
    // Abstract methods which will be
    // implemented by its subclass(es)
    abstract public void init(HardwareMap hardwareMap);
    abstract public void run(GamepadEx gamepad1, GamepadEx gamepad2);
    abstract public void runIntake(GamepadEx controller);

}
