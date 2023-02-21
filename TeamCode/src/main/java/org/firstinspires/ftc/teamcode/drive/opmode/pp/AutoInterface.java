package org.firstinspires.ftc.teamcode.drive.opmode.pp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.Cone;
import org.firstinspires.ftc.teamcode.drive.IntakeSlideSubsystemAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.LeftDoubleHigh;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.LeftDoubleMedium;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.LeftHigh;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.LeftHighMedium;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.LeftTEST;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.RightDoubleMedium;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.RightDoubleHigh;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.RightHighMedium;
import org.firstinspires.ftc.teamcode.drive.opmode.pp.autos.RightHigh;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="0-All Autos", group="Linear Opmode")
public class AutoInterface extends LinearOpMode {

    private enum Side {
        SELECTING,
        LEFT,
        RIGHT
    }
    private enum Junctions {
        SELECTING,
        LEFT_MM,
        LEFT_HH,
        LEFT_HM,
        LEFT_H,
        RIGHT_MM,
        RIGHT_HH,
        RIGHT_HM,
        RIGHT_H,
        LEFT_TEST
    }

    // Selection variables
    private boolean sideSelected = false;
    private boolean junctionsSelected = false;


    // Vuforia Variables
    int label = 0;
    int parkDistance = 1;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean coneThere = false;

    public static final String TAG = "Vuforia VuMark Sample";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AYP9k3r/////AAABmbsKp4S4+0RSpbJyMlJGbNQJWbthdpl1gIp8CO+DnDwIDkzifNXPuUMawrPbYmKwDfWtSi+PAKLOcvbHmHZxsTM24Sd32QsBy/RarvDqfIJgEIVDiUXpTlOvKCqFNCS5FGivU6Tz3C5FIhf5N/KapHhETsd2ExGtCtsZSE7QQw5SCjynKE+JvP/DnjZ8eBk6PYlS/TUdvQmonUSTkPwPCEXcL3HVO9Mw+QjvYT0eA93l7yn2NssK+37MjpJBn7kzME8FUmurwynqPJA5Ido5l/iafDl53Hndd+vl0H5ooXY0qVE1mc8HUK5lYoVXMygBDqa9Grkghg8bD791U09C20SnuKdwFCWH0Ic6zZUkeH9o";

    @Override
    public void runOpMode() throws InterruptedException  {
        Side side;
        Junctions junctions;

        side = Side.SELECTING;
        junctions = Junctions.SELECTING;
        
        // initialize autos
        // Right Side
        RightHigh rightAuto5 = null;
        RightHighMedium rightAuto4 = null;
        RightDoubleHigh rightAuto3 = null;
        RightDoubleMedium rightAuto2 = null;

        // Left Side
        LeftHigh leftAuto5 = null;
        LeftHighMedium leftAuto4 = null;
        LeftDoubleHigh leftAuto3 = null;
        LeftDoubleMedium leftAuto2 = null;

        LeftTEST leftTest = new LeftTEST();

        // Vuforia
        initVuforia();
        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables targets1 = this.vuforia.loadTrackablesFromAsset("PowerPlay1");
        VuforiaTrackables targets2 = this.vuforia.loadTrackablesFromAsset("PowerPlay2");
        VuforiaTrackables targets3 = this.vuforia.loadTrackablesFromAsset("PowerPlay3");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets1);
        allTrackables.addAll(targets2);
        allTrackables.addAll(targets3);

        // VuforiaTrackable relicTemplate = relicTrackables.get(0);
        // relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // cone
        IntakeSlideSubsystemAuto intakeSlide = new IntakeSlideSubsystemAuto();
        intakeSlide.init(hardwareMap);
        Cone cone = new Cone();
        cone.init(drive, intakeSlide, hardwareMap, this);

        while (!isStopRequested() && !sideSelected && !junctionsSelected) {
            switch (side) {
                case SELECTING:
                    if (gamepad1.left_bumper) {
                        side = Side.LEFT;
                        sideSelected = true;
                        while (!junctionsSelected) {
                            switch (junctions) {
                                case SELECTING:
                                    if (gamepad1.a) {
                                       junctions = Junctions.LEFT_HM;
                                       junctionsSelected = true;
                                    } else if (gamepad1.x) {
                                        junctions = Junctions.LEFT_MM;
                                        junctionsSelected = true;
                                    } else if (gamepad1.y) {
                                        junctions = Junctions.LEFT_HH;
                                        junctionsSelected = true;
                                    } else if (gamepad1.b) {
                                        junctions = Junctions.LEFT_H;
                                        junctionsSelected = true;
                                    } else if (gamepad1.dpad_up) {
                                        junctions = Junctions.LEFT_TEST;
                                        junctionsSelected = true;
                                    }
                            }
                            telemetry.addData("Selected Side:", side.name());
                            telemetry.addData("To select DOUBLE HIGH:", "Gamepad Y");
                            telemetry.addData("To select DOUBLE MEDIUM:", "Gamepad X");
                            telemetry.addData("To select HIGH MEDIUM:", "Gamepad A");
                            telemetry.addData("To select HIGH:", "Gamepad B");
                            telemetry.addData("Run Test:", "DPAD UP");
                            telemetry.update();
                        }

                    } else if (gamepad1.right_bumper) {
                        side = Side.RIGHT;
                        sideSelected = true;
                        while (!junctionsSelected) {
                            switch (junctions) {
                                case SELECTING:
                                    if (gamepad1.a) {
                                        junctions = Junctions.RIGHT_HM;
                                        junctionsSelected = true;
                                    } else if (gamepad1.x) {
                                        junctions = Junctions.RIGHT_MM;
                                        junctionsSelected = true;
                                    } else if (gamepad1.y) {
                                        junctions = Junctions.RIGHT_HH;
                                        junctionsSelected = true;
                                    } else if (gamepad1.b) {
                                        junctions = Junctions.RIGHT_H;
                                        junctionsSelected = true;
                                    }
                            }
                            telemetry.addData("Selected Side:", side.name());
                            telemetry.addData("To select DOUBLE HIGH:", "Gamepad Y");
                            telemetry.addData("To select DOUBLE MEDIUM:", "Gamepad X");
                            telemetry.addData("To select HIGH MEDIUM:", "Gamepad A");
                            telemetry.addData("To select HIGH:", "Gamepad B");
                            telemetry.update();
                        }
                    }
            }
            telemetry.addData("To select LEFT:", "Gamepad Left Bumper");
            telemetry.addData("To select RIGHT:", "Gamepad Right Bumper");
            if (junctionsSelected) {
                telemetry.addData("Selected Auto:", junctions.name());
            } else {
                telemetry.addData("Selected Side:", side.name());
            }
            telemetry.update();
        }

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


        //Vufrofia
        targets1.activate();  // octopus
        targets2.activate(); // triangle
        targets3.activate(); // traffic

        boolean targetVisible = false;
        String targetName = "NOT FOUND";

        runtime.reset();
        if (side == Side.RIGHT) {
            while (!isStopRequested() && !opModeIsActive()) {
                if (!targetVisible) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        if ( ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()){
                            targetVisible = true;
                            targetName = trackable.getName();
                            if (targetName == "PowerPlay2") {
                                label = 1;
                                parkDistance = 1;
                            } else if (targetName == "PowerPlay1") {
                                label = 2;
                                parkDistance = 24;
                            } else if (targetName == "PowerPlay3") {
                                label = 3;
                                parkDistance = 48;
                            }
                            break;
                        }
                    }
                }
                intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.IN);
                telemetry.addData("Visible Target", targetName);
                telemetry.addData("Lable #", label);
                telemetry.addData("Parking:", parkDistance);
                telemetry.addData("Selected Auto:", junctions.name());
                telemetry.update();
            }
        } else {
            while (!isStopRequested() && !opModeIsActive()) {
                if (!targetVisible) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        if ( ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()){
                            targetVisible = true;
                            targetName = trackable.getName();
                            if (targetName == "PowerPlay2") {
                                label = 1;
                                parkDistance = 48;
                            } else if (targetName == "PowerPlay1") {
                                label = 2;
                                parkDistance = 24;
                            } else if (targetName == "PowerPlay3") {
                                label = 3;
                                parkDistance = 1;
                            }
                            break;
                        }
                    }
                }
                intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.IN);
                telemetry.addData("Visible Target", targetName);
                telemetry.addData("Zone #", label);
                telemetry.addData("Parking:", parkDistance);
                telemetry.addData("Selected Auto:", junctions.name());
                telemetry.update();
            }
        }

        intakeSlide.setIntakePosition(IntakeSlideSubsystemAuto.IntakeState.IN);

        // this telemetry will not be seen because of the loop above
        telemetry.addData("Auto selected:", junctions.name());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;
        runtime.reset();
        switch (junctions) {
            case LEFT_MM:
                telemetry.addData("Auto:", "Left Medium Medium");
                telemetry.update();
            case LEFT_HH:
                telemetry.addData("Auto:", "Left High High");
                telemetry.update();
            case LEFT_HM:
                telemetry.addData("Auto:", "Left High Medium");
                telemetry.update();
            case LEFT_H:
                telemetry.addData("Auto:", "Left High Park");
                telemetry.update();
                //leftAuto5.followPath(drive, intakeSlide, cone, parkDistance);
            case RIGHT_MM:
                telemetry.addData("Auto:", "Left Medium Medium");
                telemetry.update();
            case RIGHT_HH:
                telemetry.addData("Auto:", "Left High High");
                telemetry.update();
            case RIGHT_HM:
                telemetry.addData("Auto:", "Left High Medium");
                telemetry.update();
            case RIGHT_H:
                telemetry.addData("Auto:", "Left High Park");
                telemetry.update();
                //rightAuto5.followPath(drive, intakeSlide, cone, parkDistance);
            case LEFT_TEST:
                telemetry.addData("Auto:", "Left Test");
                telemetry.update();
                leftTest.followPath(drive, intakeSlide, cone, parkDistance, this);

        }
        telemetry.update();
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
}
