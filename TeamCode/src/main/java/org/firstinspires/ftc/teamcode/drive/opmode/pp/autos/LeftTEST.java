package org.firstinspires.ftc.teamcode.drive.opmode.pp.autos;


import android.sax.StartElementListener;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.intakeslide.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.AutoInterface;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LeftTEST {
    enum TestSelect {
        MOVE_TO_MIDDLE_OF_ARENA,
        MOVE_TO_TOP_LEFT_MID,
        MOVE_TO_TOP_MID,
        MOVE_TO_CONE_STACK,
        BACK_INTO_MID_JUNCTION_FROM_TOP,
        MOVE_AND_SCORE_IN_HIGH_JUNCTION,
    }
    TestSelect testSelect = TestSelect.MOVE_TO_MIDDLE_OF_ARENA;
    boolean testSelected = false;

    enum DriveState {
        TRAJ_1,
        PARK,
        IDLE,
    }
    DriveState driveState = DriveState.IDLE;
    private ElapsedTime runtime = new ElapsedTime();

    LinearOpMode op;
    SampleMecanumDrive drive;
    IntakeSlideSubsystemAuto intakeSlide;
    Cone cone;
    AutoInterface positions = new AutoInterface();

    double widthOfTile = 23.5; 		//includes 0.5 of tile edge on both sides
    double widthOfTileEdge = 0.75;
    double lengthOfRobot = 13.972;
    double widthOfRobot = 13.465;

    double yStartCenterOffset = 0.5 * widthOfTile + 0.5 * lengthOfRobot - lengthOfRobot  - widthOfTileEdge;
    //Distance from back of robot to center of cone guide
    //Math: diameterOfGear + radiusOfCone + yIntakeEdgeOffset = lengthOfConeGuide
    float lengthOfConeGuide = 5;
    float diameterOfCone = 4;

    double xStart = 0;
    double yStart = 0;

    double xBesideLow = xStart;
    double yBesideLow = yStart + 1.5 * widthOfTile + yStartCenterOffset;
    double xTopMid = xStart + 0.5 * widthOfTile;
    double yTopMid = yBesideLow + 0.5 * widthOfTile;
    double xConeStack = xStart - 1.5 * widthOfTile + 0.5 * widthOfRobot + 0.5 * widthOfTileEdge;
    double yConeStack = yTopMid;
    double xTopLow = xTopMid - widthOfTile;
    double yTopLow = yTopMid;
    double xTopHigh = xTopMid + widthOfTile;
    double yTopHigh = yTopMid;
    double xBesideMid = xStart + widthOfTile;
    double yBesideMid = yBesideLow;
    double xBesideHigh = xBesideMid + widthOfTile;
    double yBesideHigh = yBesideLow;

    //Should be about 7.764”
    double distanceBackIntoJunction = lengthOfConeGuide + ((widthOfTile - 0.5*diameterOfCone) - (0.5*widthOfTile + 0.5*lengthOfRobot));

    Pose2d Start;
    Pose2d ConeStack;
    Pose2d TopLow;
    Pose2d TopMid;
    Pose2d TopHigh;
    Pose2d BesideLow;
    Pose2d BesideMid;
    Pose2d BesideHigh;

    Pose2d TopLeftMid;
    Pose2d TopLeftHigh;
    Pose2d TopRightHigh;
    Pose2d LeftMiddleArenaHigh;
    Pose2d RightMiddleArenaHigh;

    boolean previousGamepadR;
    boolean previousGamepadL;


    public void init(SampleMecanumDrive d, IntakeSlideSubsystemAuto i, Cone c, LinearOpMode o, int iSide) {
        drive = d;
        intakeSlide = i;
        cone = c;
        op = o;
        iSide = iSide < 0 ? 1:-1;

        //Set Position Coordinates
        //Top and besides are relative to the specified junction, not arena
        Start = new Pose2d(iSide * xStart, yStart , Math.toRadians(0));
        ConeStack = new Pose2d(iSide * xConeStack, yConeStack, Math.toRadians(-90));
        TopLow = new Pose2d(iSide * xTopLow, yTopLow, Math.toRadians(0));
        TopMid = new Pose2d(iSide * xTopMid, yTopMid, Math.toRadians(0));
        TopHigh = new Pose2d(iSide * xTopHigh, yTopHigh, Math.toRadians(0));
        BesideLow = new Pose2d(iSide * xBesideLow, yBesideLow, Math.toRadians(0));
        BesideMid = new Pose2d(iSide * xBesideMid, yBesideMid, Math.toRadians(0));
        BesideHigh = new Pose2d(iSide * xBesideHigh, yBesideHigh, Math.toRadians(0));

        TopLeftMid = new Pose2d(iSide * xBesideLow, yTopMid, Math.toRadians(0));
        TopLeftHigh = new Pose2d(iSide * xBesideMid, yTopHigh, Math.toRadians(0));
        TopRightHigh = new Pose2d(iSide * xBesideHigh, yTopHigh, Math.toRadians(0));
        LeftMiddleArenaHigh = new Pose2d(iSide * xBesideLow, yBesideLow+widthOfTile, Math.toRadians(0));
        RightMiddleArenaHigh = new Pose2d(iSide *  xBesideMid, yBesideMid+widthOfTile, Math.toRadians(0));

    }

    public void followPath(int parkDistance) {
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .forward(50)
                .strafeRight(8)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajSeq1.end())
                .strafeRight(16)
                .strafeLeft(parkDistance)
                .build();

        if (op.isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq1);
        cone.drop = true;
        runtime.reset();
        while (runtime.seconds() < 2) {
            // wait.. add telemetry here
            op.telemetry.addData("Waiting", "to align");
            op.telemetry.update();
        }
        cone.align(IntakeSlideSubsystemAuto.LiftState.MEDIUM, false);
        drive.followTrajectorySequence(park);
        intakeSlide.runToREST();
    }

    public void followPath2() {
        Pose2d pickUp = positions.LeftConeStack;
        Pose2d dropOff = positions.Medium;

        drive.setPoseEstimate(pickUp);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(pickUp)
                .lineToLinearHeading(dropOff)
                .build();
        if (op.isStopRequested()) return;
        runtime.reset();
        drive.followTrajectorySequence(trajSeq1);
        // reset slides to rest
        intakeSlide.runToREST();
    }
    public void runOtherTests(Gamepad gamepad1) {
        if (op.isStopRequested()) return;
        while (!testSelected && op.opModeIsActive()) {
            if (gamepad1.a) {
                testSelected = true;
            }
            switch (testSelect) {
                case MOVE_TO_MIDDLE_OF_ARENA:
                    // test 1
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "1");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.MOVE_TO_TOP_LEFT_MID;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.MOVE_AND_SCORE_IN_HIGH_JUNCTION;
                    }
                    break;
                case MOVE_TO_TOP_LEFT_MID:
                    // test 2
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "2");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.MOVE_TO_TOP_MID;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.MOVE_TO_MIDDLE_OF_ARENA;
                    }
                    break;
                case MOVE_TO_TOP_MID:
                    // test 3
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "3");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.MOVE_TO_CONE_STACK;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.MOVE_TO_TOP_LEFT_MID;
                    }
                    break;
                case MOVE_TO_CONE_STACK:
                    // test 4
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "4");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.BACK_INTO_MID_JUNCTION_FROM_TOP;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.MOVE_TO_TOP_MID;
                    }
                    break;
                case BACK_INTO_MID_JUNCTION_FROM_TOP:
                    // test 5
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "5");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.MOVE_AND_SCORE_IN_HIGH_JUNCTION;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.MOVE_TO_CONE_STACK;
                    }
                    break;
                case MOVE_AND_SCORE_IN_HIGH_JUNCTION:
                    // test 6
                    previousGamepadR = gamepad1.right_bumper;
                    previousGamepadL = gamepad1.left_bumper;
                    op.telemetry.addData("Test", "6");
                    if (gamepad1.right_bumper != previousGamepadR) {
                        testSelect = TestSelect.MOVE_TO_MIDDLE_OF_ARENA;
                    } else if (gamepad1.left_bumper != previousGamepadL) {
                        testSelect = TestSelect.BACK_INTO_MID_JUNCTION_FROM_TOP;
                    }
                    break;
            }
            op.telemetry.addData("To Cycle Tests:", "Right/Left Bumper");
            op.telemetry.addData("Test Selected:", testSelect.name());
            op.telemetry.addData("To End Selection:", "Gamepad A");
            op.telemetry.update();
        }
        op.telemetry.addData("Test Selected:", testSelect.name());
        switch (testSelect) {
            case MOVE_TO_MIDDLE_OF_ARENA:
                op.telemetry.addData("Test:", "Push robot while it is moving to test the robot ability to realign itself on path to destination");
                op.telemetry.update();
                MoveToMiddleOfArena();
                break;
            case MOVE_TO_TOP_LEFT_MID:
                op.telemetry.update();
                MoveToTopLeftMid();
                break;
            case MOVE_TO_TOP_MID:
                op.telemetry.update();
                MoveToTopMid();
                break;
            case MOVE_TO_CONE_STACK:
                op.telemetry.addData("Test:", "Do not preload cone at start");
                op.telemetry.update();
                MoveToConeStack();
                break;
            case BACK_INTO_MID_JUNCTION_FROM_TOP:
                op.telemetry.update();
                BackIntoMidJunctionFromTop();
                break;
            case MOVE_AND_SCORE_IN_HIGH_JUNCTION:
                op.telemetry.update();
                MoveAndScoreHighJunction();
                break;
        }
    }

    // MEDIUM velocity constraint is about 60%.
    // SLOW velocity constraint is about 40%.
    /**
     * Test 1 for roadrunner capabilities
     */
    // finished
    private void MoveToMiddleOfArena() {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                // set to 40%
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_SLOW, SampleMecanumDrive.ACCEL_CONSTRAINT_SLOW) // max speed
                .lineToLinearHeading(LeftMiddleArenaHigh)
                .resetConstraints()
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToPICKUP();
        drive.followTrajectorySequence(traj1);
        // reset slides to rest
        intakeSlide.runToREST();
    }

    /**
     * Test 2 for roadrunner capabilities
     */
    // finished
    private void MoveToTopLeftMid() {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                // set to 60%
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(TopLeftMid)
                .resetConstraints()
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToLOW();
        drive.followTrajectorySequence(traj1);
        // reset slides to rest
        intakeSlide.runToREST();
    }

    /**
     * Test 3 for roadrunner capabilities
     */
    // finished
    private void MoveToTopMid() {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(TopLeftMid)
                .resetConstraints()
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(TopLeftMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(TopMid)
                .resetConstraints()
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToLOW();
        drive.followTrajectorySequence(traj1);
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj2);
        // reset slides to rest
        intakeSlide.runToREST();
    }

    /**
     * Test 4 for roadrunner capabilities
     */
    private void MoveToConeStack() {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(TopLeftMid)
                .lineToLinearHeading(ConeStack)
                .resetConstraints()
                .build();

        intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.OUT);
        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToPICKUP2();
        drive.followTrajectorySequence(traj1);
        // fails to intake because intake position is already set to IN
        cone.simplePickUp();
    }

    /**
     * Test 5 for roadrunner capabilities
     */
    private void BackIntoMidJunctionFromTop() {
        drive.setPoseEstimate(TopMid);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(TopMid)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_HALF, SampleMecanumDrive.ACCEL_CONSTRAINT_HALF) // max speed
                .lineToLinearHeading(new Pose2d(xTopMid, yTopMid-distanceBackIntoJunction, Math.toRadians(0)))
                .lineToLinearHeading(TopMid)
                .resetConstraints()
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToMEDIUM();
        drive.followTrajectorySequence(traj1);
        // reset slides to rest
        intakeSlide.runToREST();
    }

    /**
     * Test 6 for roadrunner capabilities
     */
    private void MoveAndScoreHighJunction() {
        drive.setPoseEstimate(Start);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(Start)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_MEDIUM, SampleMecanumDrive.ACCEL_CONSTRAINT_MEDIUM) // max speed
                .lineToLinearHeading(TopLeftMid)
                .resetConstraints()
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(Start)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_SLOW, SampleMecanumDrive.ACCEL_CONSTRAINT_SLOW) // max speed
                .lineToLinearHeading(new Pose2d(positions.startSide*xBesideLow*-1, yBesideLow+widthOfTile, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(xBesideLow+distanceBackIntoJunction, yBesideLow+widthOfTile, Math.toRadians(-90)))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(Start)
                .setTurnConstraint(DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .setConstraints(SampleMecanumDrive.VEL_CONSTRAINT_SLOW, SampleMecanumDrive.ACCEL_CONSTRAINT_SLOW) // max speed
                .lineToLinearHeading(LeftMiddleArenaHigh)
                .build();

        if (op.isStopRequested()) return;
        runtime.reset();
        intakeSlide.runToLOW();
        drive.followTrajectorySequence(traj1);
        intakeSlide.runToHIGH();
        drive.followTrajectorySequence(traj2);
        cone.simpleDropOff();
        drive.followTrajectorySequence(traj3);
        // reset slides to rest
        intakeSlide.runToREST();
    }
}
