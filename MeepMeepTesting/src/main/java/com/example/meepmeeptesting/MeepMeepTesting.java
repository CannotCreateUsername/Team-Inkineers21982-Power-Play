//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public MeepMeepTesting() {
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = (new DefaultBotBuilder(meepMeep)).setConstraints(60.0D, 60.0D, Math.toRadians(180.0D), Math.toRadians(180.0D), 15.0D).followTrajectorySequence((drive) -> {
            return drive.trajectorySequenceBuilder(new Pose2d(-60.0D, -34.0D, 0.0D)).lineTo(new Vector2d(-60.0D, -15.0D)).splineTo(new Vector2d(-9.0D, -9.0D), Math.toRadians(45.0D)).splineToConstantHeading(new Vector2d(-30.0D, -60.0D), Math.toRadians(45.0D)).build();
        });
        meepMeep.setBackground(Background.FIELD_POWERPLAY_OFFICIAL).setDarkMode(true).setBackgroundAlpha(0.95F).addEntity(myBot).start();
    }
}
