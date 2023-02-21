package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Left Test", group="Linear Opmode")
public class LeftTEST extends LinearOpMode {
    enum DriveState {
        TRAJECTORY1,
        DROP_OFF,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // VuforiaTrackable relicTemplate = relicTrackables.get(0);
        // relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        boolean coneThere = false;

        int label = 0;
        int parkDistance = 1;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // cone
        IntakeSlideSubsystemAuto intakeSlide = new IntakeSlideSubsystemAuto();
        intakeSlide.init(hardwareMap);
        Cone cone = new Cone();
        cone.init(drive, intakeSlide, hardwareMap, this);

//        // intake
//        IntakeSlideSubsystem2 intakeSlide2 = new IntakeSlideSubsystem2();
//        intakeSlide2.init(hardwareMap);

         /*
            read: Trajectories Overview
            https://learnroadrunner.com/trajectories.html#trajectories-vs-paths

            benefit of sequencer
            https://learnroadrunner.com/trajectory-sequence.html#overview

            reminder:
            Keep this in mind as the turn function will go counter-clockwise.
            all angles are in radian
            each tile is 24 inches (2 feet) ref: https://learnroadrunner.com/trajectories.html#coordinate-system
            always call drive.setPoseEstimate(startPose) to orient the drive ;

         */

        // we assume A2/F5 is starting point, the robot back is facing the wall
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        // run to bottom high junction
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .strafeLeft(24)
                .forward(48)
                .strafeLeft(8)
                .addTemporalMarker(() -> {
                    intakeSlide.liftState = IntakeSlideSubsystemAuto.LiftState.PICKUP2;
                    intakeSlide.run();
                })
                .waitSeconds(0.5)
                .resetConstraints()
                .build();


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        runtime.reset();

        drive.followTrajectorySequence(trajSeq);

        // the last thing auto should do is move slide back to rest
        telemetry.update();
    }

    public void followPath(SampleMecanumDrive drive, IntakeSlideSubsystemAuto intakeSlide, Cone cone, int parkDistance) {
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

        if (isStopRequested()) return;
        driveState = DriveState.TRAJECTORY1;
        drive.followTrajectorySequenceAsync(trajSeq1);

        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            switch (driveState) {
                case TRAJECTORY1:
                    if (!drive.isBusy()) {
                        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
                        driveState = DriveState.DROP_OFF;
                    }
                case DROP_OFF:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(park);
                        driveState = DriveState.PARK;
                    }
            }
            telemetry.update();
        }
    }
}