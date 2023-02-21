package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LeftTEST {
    enum DriveState {
        TRAJECTORY1,
        DROP_OFF,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();

    public void followPath(SampleMecanumDrive drive, IntakeSlideSubsystemAuto intakeSlide, Cone cone, int parkDistance, LinearOpMode op) {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(50)
                .strafeRight(8)
                .resetConstraints()
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(16)
                .strafeLeft(parkDistance)
                .build();

        if (op.isStopRequested()) return;
        driveState = DriveState.TRAJECTORY1;
        drive.followTrajectorySequenceAsync(trajSeq1);

        runtime.reset();
        while (op.opModeIsActive() && !op.isStopRequested()) {
            switch (driveState) {
                case TRAJECTORY1:
                    if (!drive.isBusy()) {
                        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
                        driveState = DriveState.DROP_OFF;
                    }
                case DROP_OFF:
                    if (runtime.seconds() > 2) {
                        while (cone.loaded && op.opModeIsActive()) {
                            // wait for cone alignment process to be done
                        }
                    }
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(park);
                        driveState = DriveState.PARK;
                    }
                case PARK:
                    if (!drive.isBusy()) {
                        driveState = DriveState.IDLE;
                    }
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            op.telemetry.addData("Status:", "Running");
            op.telemetry.update();
        }
    }
}