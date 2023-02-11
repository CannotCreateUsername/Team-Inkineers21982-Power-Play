/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Red & Blue Right Encoder", group = "Concept")
@Disabled
public class AaRightSideE extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor slides = null;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    //Encoder Stuff
    static final double COUNTS_PER_MOTOR_REV = 312;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_DIAMETER_INCHES = 3.77;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES *3.14159265359);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYP9k3r/////AAABmbsKp4S4+0RSpbJyMlJGbNQJWbthdpl1gIp8CO+DnDwIDkzifNXPuUMawrPbYmKwDfWtSi+PAKLOcvbHmHZxsTM24Sd32QsBy/RarvDqfIJgEIVDiUXpTlOvKCqFNCS5FGivU6Tz3C5FIhf5N/KapHhETsd2ExGtCtsZSE7QQw5SCjynKE+JvP/DnjZ8eBk6PYlS/TUdvQmonUSTkPwPCEXcL3HVO9Mw+QjvYT0eA93l7yn2NssK+37MjpJBn7kzME8FUmurwynqPJA5Ido5l/iafDl53Hndd+vl0H5ooXY0qVE1mc8HUK5lYoVXMygBDqa9Grkghg8bD791U09C20SnuKdwFCWH0Ic6zZUkeH9o";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        front_left = hardwareMap.get(DcMotor.class, "left_front_drive");
        back_left = hardwareMap.get(DcMotor.class, "left_back_drive");
        front_right = hardwareMap.get(DcMotor.class, "right_front_drive");
        back_right = hardwareMap.get(DcMotor.class, "right_back_drive");
        slides = hardwareMap.get(DcMotor.class, "slides");

        //Encoder Stuff
        resetPosition();
        modeUseEncoders();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            int label = 0;
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            while (runtime.time() < 8 && opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            if (recognition.getLabel() == "1 Bolt") {
                                label = 1;
                            } else if (recognition.getLabel() == "2 Bulb") {
                                label = 2;
                            } else if (recognition.getLabel() == "3 Panel") {
                                label = 3;
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
            if (label == 1) {
                dropCone();
                strafeLeft(1000);
            } else if (label == 2) {
                dropCone();
            } else if (label == 3) {
                dropCone();
                strafeRight(1000);
            } else {
                goForward(1000);
                telemetry.addData("Haha","No image detected");
            }
        }
    }

    //Encoder Strafing
    private void encoderStrafeRight(int distanceInches) {
        encoderDrive(DRIVE_SPEED, distanceInches, -distanceInches, -distanceInches, distanceInches, 2);
    }
    private void encoderStrafeLeft(int distanceInches) {
        encoderDrive(DRIVE_SPEED, -distanceInches, distanceInches, distanceInches, -distanceInches, 2);
    }
    private void encoderForward(int distanceInches) {
        encoderDrive(DRIVE_SPEED, distanceInches, distanceInches, distanceInches, distanceInches, 2);
    }
    private void encoderBackward(int distanceInches) {
        encoderDrive(DRIVE_SPEED, -distanceInches, -distanceInches, -distanceInches, -distanceInches, 2);
    }

    //Encoder Running
    public void encoderDrive(double speed, double frontLeftInches, double backLeftInches, double frontRightInches, double backRightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = front_left.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newBackLeftTarget = back_left.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = front_right.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackRightTarget = back_right.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            front_left.setTargetPosition(newFrontLeftTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_right.setTargetPosition(newBackRightTarget);

            modeRunToPosition();
            //Motor go bRr

            front_left.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (front_left.isBusy() && back_left.isBusy() && front_right.isBusy() && back_right.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Never gonna", "give you up");
                telemetry.addData("Never gonna", "let you down");
                telemetry.addData("Never gonna", "turn around");
                telemetry.addData("and", "desert you");
                telemetry.update();
            }
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    private void highJunction() {
        slides.setTargetPosition(2000);
        slides.setPower(0.5);
        while (opModeIsActive() && slides.isBusy()) {
            telemetry.addData("Dispenser", "Going up!");
        }
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void mediumJunction () {
        slides.setTargetPosition(1200);
        slides.setPower(0.5);
        while (opModeIsActive() && slides.isBusy()) {
            telemetry.addData("Dispenser", "Going up!");
        }
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void lowJunction() {
        slides.setTargetPosition(500);
        slides.setPower(0.5);
        while (opModeIsActive() && slides.isBusy()) {
            telemetry.addData("Dispenser", "Going up!");
        }
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void dropCone() {
        encoderStrafeLeft(10);
        encoderForward(15);
        encoderStrafeRight(5);
        //insert arm code here
    }

    private void resetPosition() {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void modeRunToPosition() {
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void modeUseEncoders() {
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }
    private void motorBusy() {
        front_right.isBusy();
        back_right.isBusy();
        front_left.isBusy();
        back_left.isBusy();
    }

    //Extra Hardcode
    private void stop2(int sleepTime) {
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
        sleep(sleepTime);
    }
    private void goBackward(int sleepTime) {
        front_right.setPower(-0.2);
        back_right.setPower(-0.2);
        front_left.setPower(-0.2);
        back_left.setPower(-0.2);
        sleep(sleepTime);
    }
    private void goForward(int sleepTime) {
        front_right.setPower(0.3);
        back_right.setPower(0.3);
        front_left.setPower(0.3);
        back_left.setPower(0.3);
        sleep(sleepTime);
    }
    private void strafeRight(int sleepTime) {
        front_right.setPower(-0.5);
        back_right.setPower(0.5);
        front_left.setPower(0.5);
        back_left.setPower(-0.5);
        sleep(sleepTime);
    }
    private void strafeLeft(int sleepTime) {
        front_right.setPower(0.5);
        back_right.setPower(-0.5);
        front_left.setPower(-0.5);
        back_left.setPower(0.5);
        sleep(sleepTime);
    }
}

