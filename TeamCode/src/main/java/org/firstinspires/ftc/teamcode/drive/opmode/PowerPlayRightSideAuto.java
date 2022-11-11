package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.IntakeSlide;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystem2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Auto Right Simple", group="Linear Opmode")
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
        strafe(drive , -24);

        // go to certain location
        while (intakeSlide2.slides.isBusy()){
            intakeSlide2.runToPosition(intakeSlide2.targetPositionPickup, 0.3);
        }



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("X", poseEstimate.getX());
        telemetry.addData("Y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        telemetry.update();
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

        double timeLimit = Math.abs(distance) / 8;

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
}