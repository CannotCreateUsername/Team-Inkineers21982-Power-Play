package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;





public class GamepadHelper {

    public enum GamePadState {
        POSITIVE,
        NETURAL,
        NEGATIVE
    }

    private    double  minMultiplier = 0.1;
    private         double  maxMultiplier = 0.75;
    private         double  incrementMultipler = 0.1;
    private        double gameStickMultipler;
    private  double  timeIncrementInMs = 200;

    /*
    Time Base Ramping
    If the driver keep on pressing the gamepad, the sensitivity of the gamepad input will increase over time
    */
    ElapsedTime xGamePadTimer; 
    boolean isRamping;
    GamePadState prevoiusGameStickState ;
    GamePadState currentGameStickState ;

    
    public void init(){
        prevoiusGameStickState = GamePadState.NETURAL;
        currentGameStickState = GamePadState.NETURAL;
        gameStickMultipler = minMultiplier;
        xGamePadTimer = new ElapsedTime();
        isRamping = false;
    }

    /**
     *  OLD LOGIC:
     *  Ramp is true  when 
     *  1. gamePad.x change from 0 to any positive
     *  2. gamePad.x change from 0 any negative 
     *  3. gamePad.x change from any negative to any positive
     *  4. gamePad.x change from any positive to any negative 
     *
     *  assumption:  3 and 4 are covered by 1 and 2 
     */
    /**
     if (previousXGamePad == 0 && myGameStick != 0 ){
     isRamping = true;
     xgameStickMultipler = minMultiplier;
     xGamePadTimer.reset();
     }
     previousXGamePad = myGameStick;
     */

    /**
     Ivan: this previous logic assume that we can capture the value of gamePad = 0.
     But since the moving of gamepad joystick is fast, we might not be able to store the value of 0 reliability

     so here is the new logic:
     Ramp is true  when there is any chagne of the game pad state
     i.e. from NEUTRAL TO NEGATIVE , NEUTRAL TO POSISTIVE, POSTIVE TO NEGATIVE, NEGATIVE TO POSITIVE ....
     */
    
    public double  getGamepadStickRampingMultiplier(float gameStick){

        prevoiusGameStickState = currentGameStickState;
        if (gameStick < 0){
            currentGameStickState = GamePadState.NEGATIVE;
        } else  if (gameStick > 0){
            currentGameStickState = GamePadState.POSITIVE;
        } else {
            currentGameStickState = GamePadState.NETURAL;
            gameStickMultipler = minMultiplier;
        }
        if (prevoiusGameStickState != currentGameStickState ){
            isRamping = true;
            gameStickMultipler = minMultiplier;
            xGamePadTimer.reset();
        }

        if (isRamping && xGamePadTimer.milliseconds() > timeIncrementInMs) {
            if (gameStickMultipler <= maxMultiplier) {
                gameStickMultipler += incrementMultipler;
            } else {
                isRamping = false;
            }

            if (gameStickMultipler > maxMultiplier) {
                gameStickMultipler = maxMultiplier;
            }

            xGamePadTimer.reset();
        }

        return gameStickMultipler; 
        
    }






    /**
     *  old code with 2 step ramping, for postperity
     *
     */
    /**
     double maxMultiplier = 0.75;
     double minMultiplier = 0.25;
     double threshHold1 = 0.75;

     double xgameStickMultipler = 0;
     double yGampePadMultipler = 0;
     */

    /**
     * Two (X)  step ramp
     *
     * If gamepad.x is less than 0.5 , divide the power by a fixed amount (ReductionFactor)
     * Else , go the max power
     */
            /*
            if (  Math.abs(myGameStick) < threshHold1  ){
                xgameStickMultipler = minMultiplier;
            } else {
                xgameStickMultipler = maxMultiplier;
            }

            if (  Math.abs(myGamePad.left_stick_y) < threshHold1  ){
                yGampePadMultipler = minMultiplier;
            } else {
                yGampePadMultipler = maxMultiplier;
            }

             */

            /*
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -myGamePad.left_stick_y*yGampePadMultipler,// -myGamePad.left_stick_y*0.75,
                            -myGameStick*xgameStickMultipler,// -myGameStick*0.75,
                            -myGamePad.right_stick_x
                    )
            );

             */

}
