package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Auto Right Simple", group="Linear Opmode")
@Disabled
public class PowerPlayRightSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException  {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("init", "Drive creation completed");
        telemetry.addData("wait to start", "");

        // intake
        IntakeSlideSubsystem2 intakeSlide2 = new IntakeSlideSubsystem2();
        intakeSlide2.init(hardwareMap);


        waitForStart();
        telemetry.addData("starting", "");

        if(isStopRequested()) return;
        /** Simple Auto:
         Strafe left 24 inches
         */
        telemetry.addData("before strafe", "");
        // strafe
        // straight(drive, 48);
        // strafe(drive , -24);

        // turn(drive, -45);
        // sleep(500);
        // turn(drive, -45);


        moveSlide(intakeSlide2, intakeSlide2.targetPositionHigh, 10);
        spinIntake(intakeSlide2, IntakeSlide.IntakeState.OUT, 3);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("X", poseEstimate.getX());
        telemetry.addData("Y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        telemetry.update();


        // the last thing auto should do is move slide back to rest
        moveSlide(intakeSlide2, intakeSlide2.targetPositionRest, 30);

    }

    /**
     *
     * @param distance positive = right; negative = left (measured in inches)
     */
    private void strafe (SampleMecanumDrive drive, double distance) {

        double leftYControl = 0 ;
        double leftXControl;
        double rightXControl = 0;

        if (distance > 0 ){
            leftXControl = 0.3;
        } else {
            leftXControl = -0.3;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 16;

        while (controlTimer.seconds() < timeLimit){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl  ,
                            -leftXControl  ,
                            -rightXControl
                    )
            );
            drive.update();
        }

    }

    /**
     *
     * @param drive
     * @param distance is measured in inches
     */
    private void straight (SampleMecanumDrive drive, double distance){



        double leftYControl = 0 ;
        double leftXControl = 0 ;
        double rightXControl = 0;

        if (distance < 0 ){
            leftYControl = 0.3;
        } else {
            leftYControl = -0.3;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 16;

        while (controlTimer.seconds() < timeLimit) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl,
                            -leftXControl,
                            -rightXControl
                    )
            );
            drive.update();

        }
    }

    /**
     *
     * @param drive implements the mecanum sample so we can use Pose2d aka Roadrunner
     * @param angle Tells us how many degrees you want to turn.
     *          Positive = counterclockwise. Negative = clockwise
     */
    private void turn (SampleMecanumDrive drive, double angle){

        double leftYControl = 0 ;
        double leftXControl = 0 ;
        double rightXControl = 0;

        if (angle < 0 ){
            rightXControl = 0.3;
        } else {
            rightXControl = -0.3;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(angle) / 64;

        while (controlTimer.seconds() < timeLimit) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -leftYControl,
                            -leftXControl,
                            -rightXControl
                    )
            );
            drive.update();

        }
    }

    /**
     *
     * @param slides
     * @param inOrOut
     * @param timeoutS
     */
    public void spinIntake(IntakeSlide slides, IntakeSlide.IntakeState inOrOut, double timeoutS){
        ElapsedTime runtime = new ElapsedTime();

        // NOTE all while loop in op mode should check for
        // opModeIsActive
        while ( opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {

            // run and  keep the position until timeout
            slides.setIntakePower(inOrOut);

            // Display it for the driver.
            telemetry.addData("Intake Spinning",  inOrOut.name());
            telemetry.update();
        }
        // stop spinning
        slides.setIntakePower(IntakeSlide.IntakeState.STOP);

    }

    /**
     *
     * @param slides
     * @param position
     * @param timeoutS
     */
    public void moveSlide(IntakeSlide slides, int position, double timeoutS){

        ElapsedTime runtime = new ElapsedTime();


        // NOTE all while loop in op mode should check for
        // opModeIsActive
        while ( opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {

            // run and  keep the position until timeout
            slides.runToPosition(position);

            // Display it for the driver.
            telemetry.addData("Slide to",  " %7d", position);
            telemetry.update();
        }
    }


}