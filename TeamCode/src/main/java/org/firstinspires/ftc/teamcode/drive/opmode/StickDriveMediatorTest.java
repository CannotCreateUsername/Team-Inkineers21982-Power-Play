package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cv.CVMaster;
import org.firstinspires.ftc.teamcode.cv.StickDriveMediator;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "StickDriveMediatorTest")

public class StickDriveMediatorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


//        initialize camera,  pipeline and distance sensor
        StickDriveMediator stickDrive = new StickDriveMediator(this);
        stickDrive.setDrive(drive);
//      call the function to startStreaming
        stickDrive.observeStick();


        waitForStart();

        while (opModeIsActive()) {

            // double error = stickDrive.alignStickLateral(0.5);
            // telemetry.addData("Error", error);
            // telemetry.update();

            stickDrive.alignStick(2, 2);

        }

//        stopStreaming
        stickDrive.stopCamera();
    }
}

