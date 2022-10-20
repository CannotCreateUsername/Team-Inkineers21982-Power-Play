package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSlideSubsystem {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor slides = null;

    // 2022-10-19: THIS NUMBER MUST BE CHANGED TO MATCH ACTUAL HIEGHT!!!!!!!
    private final int targetPositionHigh = 2000;
    private final int targetPositionMedium = 1500;
    private final int targetPositionLow = 1000;
    private final int targetPositionRetract = 100;
    private final int targetPositionPickup = 0;  // assume pickup position is same as start

    // distance error factor
    // https://gm0.org/en/latest/docs/software/concepts/control-loops.html?highlight=pid#built-in-pid-controller
    // 2022-10-19: THIS NUMBER MIGHT NEED TO BE TUNED !!!
    private final double Kp = .05;

    // 2022-10-19: THE DEFAULT POWER MIGHT NEED TO BE BE TUNED !!!
    private double defaultPower = 0.7;
    private double currentPower;
    private int currentTarget;
    private String currentCaption;
    private String currentStatus;
    private LiftState liftState;

    // 2022-10-19: REVIEW THE STATE !!!
    public enum LiftState {
        START,   // all the way to the bottom
        PICKUP,  // level where the robot will pickup the cone
        RETRACT, // after picking up the cone, raise slightly so we can travel
        LOW,
        MEDIUM,
        HIGH,
    }

    public IntakeSlideSubsystem(HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slides = hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();

        currentCaption = "Lift Status";
        currentStatus = "Initialized";
        currentTarget = 0;
        currentPower = 0;
        liftState = LiftState.START;


    }

    private void runToPosition(int position, double power){
        currentTarget = position;
        slides.setTargetPosition(currentTarget);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    private void setPower(){
        if (Math.abs(slides.getCurrentPosition()- currentTarget) > 10){
            // our threshold is within
            // 10 encoder ticks of our target.
            // this is pretty arbitrary, and would have to be
            // tweaked for each robot.
            currentPower = defaultPower;
            currentStatus = "Going to: " + currentTarget;
        } else {
            double posErr = currentTarget - slides.getCurrentPosition(); // measure error in terms of distance between current position and target
            slides.setPower(posErr * Kp); //instead of fixed power, use the concept of PID and increase power in proportion with the error
            currentStatus = "Holding at: " + slides.getCurrentPosition();
        }
    }

    public void run(Gamepad gamepad1, Gamepad gamepad2){

        switch (liftState) {
            case START:
                if (gamepad1.y) {
                    // y is pressed to to High postion
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                } else if (gamepad1.x) {
                    // x is pressed, go to low position
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                }
                break;
            case RETRACT:
                if (gamepad1.y) {
                    // y is pressed to to High postion
                    currentTarget = targetPositionHigh;
                    liftState = LiftState.HIGH;
                } else if (gamepad1.x) {
                    // x is pressed, go to low position
                    currentTarget = targetPositionLow;
                    liftState = LiftState.LOW;
                } else {
                    setPower();
                }
                break;
            case HIGH:
                if (gamepad1.b){
                    // from high to extract state
                    currentTarget = targetPositionRetract;
                    liftState = LiftState.RETRACT;
                }  else {
                    setPower();
                }
                break;
            case LOW:
                if (gamepad1.b){
                    // from low to extract state
                    currentTarget = targetPositionRetract;
                    liftState = LiftState.RETRACT;
                } else {
                    setPower();
                }
                break;

        }

        runToPosition(currentTarget, currentPower);


    }

}
