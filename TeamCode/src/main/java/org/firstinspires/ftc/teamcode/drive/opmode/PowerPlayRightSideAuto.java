package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        waitForStart();
        telemetry.addData("starting", "");

        if(isStopRequested()) return;
        /** Simple Auto:
         Strafe left 24 inches
         */
        telemetry.addData("before strafe", "");
        strafe(drive , -24);


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

        double yControl = 0 ;
        double xControl;

        if (distance > 0 ){
            xControl = 0.3;
        } else {
            xControl = -0.3;
        }
        ElapsedTime controlTimer = new ElapsedTime();

        double timeLimit = Math.abs(distance) / 8;

        while (controlTimer.seconds() < timeLimit){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -yControl  ,
                            -xControl  ,
                            -xControl
                    )
            );
            drive.update();
        }

    }
}