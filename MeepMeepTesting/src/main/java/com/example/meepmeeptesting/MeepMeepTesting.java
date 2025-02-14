package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        myBot.setDimensions(16, 17);

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -62, Math.toRadians(270)))
                // SPECIMEN PATH (Sweeping) (new Pose2d(0, -62, Math.toRadians(270)))
                // Drive To Bar And Score Specimen #1
                .strafeTo(new Vector2d(-6, -30))

                .waitSeconds(0.25)

                // Drive To Collect Samples
                .setTangent(5)
                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(55)), .8)
                .waitSeconds(0.3)

                // Collect Sample #1
                .strafeToLinearHeading(new Vector2d(32, -42), Math.toRadians(-45))

                // Collect Sample #2
                .strafeToLinearHeading(new Vector2d(32, -40), Math.toRadians(45))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(36, -42), Math.toRadians(-45))

                // Collect Sample #3
                .strafeToLinearHeading(new Vector2d(40, -42), Math.toRadians(45))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(-45))

                // Collect Specimen #2
                .setTangent(3)
                .splineToLinearHeading(new Pose2d(40, -58, Math.toRadians(90)), 4.5)
                .strafeTo(new Vector2d(40, -62))

                .waitSeconds(0.15)

                // Score Specimen #2
                .strafeTo(new Vector2d(40, -58))
                .strafeTo(new Vector2d(-3, -33))

                .waitSeconds(0.25)

                // Collect Specimen #3
                .strafeTo(new Vector2d(-3, -36))
                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))
                .strafeTo(new Vector2d(40, -62))

                .waitSeconds(0.15)

                // Score Specimen #3
                .strafeTo(new Vector2d(40, -58))
//                .strafeToLinearHeading(new Vector2d(0, -36), Math.toRadians(270))
                .strafeTo(new Vector2d(0, -33))

                .waitSeconds(0.25)

                // Collect Specimen #4
                .strafeTo(new Vector2d(0, -36))
                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))
                .strafeTo(new Vector2d(40, -62))

                .waitSeconds(0.15)

                // Score Specimen #4
                .strafeTo(new Vector2d(40, -58))
//                .strafeToLinearHeading(new Vector2d(3, -36), Math.toRadians(270))
                .strafeTo(new Vector2d(3, -33))

                .waitSeconds(0.25)

                // Collect Specimen #5
                .strafeTo(new Vector2d(3, -36))
                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))
                .strafeTo(new Vector2d(40, -62))

                .waitSeconds(0.15)

                // Score Specimen #5
                .strafeTo(new Vector2d(40, -58))
//                .strafeToLinearHeading(new Vector2d(6, -36), Math.toRadians(270))
                .strafeTo(new Vector2d(6, -33))

                .waitSeconds(0.25)

                .build());


//                // SPECIMEN PATH (4 Spec (Driving)) (new Pose2d(0, -62, Math.toRadians(270)))
//                // Drive To Bar And Score Specimen #1
//                .strafeTo(new Vector2d(-6, -30))
//                .waitSeconds(0.5)
//
//                // Move to Sample Collection Zone
//                .setTangent(5)
//                .splineToLinearHeading(new Pose2d(24, -40, Math.toRadians(90)), .25)
//
//                // Collect Sample #1
//                .splineToConstantHeading(new Vector2d(48, -9), .5)
//                .strafeTo(new Vector2d(48, -50))
//                .waitSeconds(0.5)
//
//                // Collect Sample #2
//                .setTangent(1.5)
//                .splineToConstantHeading(new Vector2d(56, -9), .5)
//                .strafeTo(new Vector2d(56, -50))
//                .waitSeconds(0.5)
//
//                // Collect Specimen #2
//                .setTangent(3)
//                .splineToConstantHeading(new Vector2d(40, -62), 4.5)
//                .waitSeconds(0.5)
//
//                // Score Specimen #2
//                .splineToLinearHeading(new Pose2d(-2, -36, Math.toRadians(270)), 1.5)
//                .strafeTo(new Vector2d(-2, -30))
//                .waitSeconds(0.5)
//
//                // Collect Specimen #3
//                .strafeTo(new Vector2d(-2, -36))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))
//                .strafeTo(new Vector2d(40, -62))
//                .waitSeconds(0.5)
//
//                // Score Specimen #3
//                .splineToLinearHeading(new Pose2d(2, -36, Math.toRadians(270)), 1.5)
//                .strafeTo(new Vector2d(2, -30))
//                .waitSeconds(0.5)
//
//                // Collect Specimen #4
//                .strafeTo(new Vector2d(2, -36))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90))
//                .strafeTo(new Vector2d(40, -62))
//                .waitSeconds(0.5)
//
//                // Score Specimen #4
//                .splineToLinearHeading(new Pose2d(6, -36, Math.toRadians(270)), 1.5)
//                .strafeTo(new Vector2d(6, -30))
//                .waitSeconds(0.5)
//
//                // Park
//                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(90)), .1)
//
//                .build());


//                // SAMPLE PATH (4 Samples) (new Pose2d(-39, -62, Math.toRadians(90)))
//                // Drive To Bucket And Score Sample
//                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(-135))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(-60, -60))
//                .waitSeconds(2)
//
//                // Collect Sample #1
//                .strafeToLinearHeading(new Vector2d(-48, -50), Math.toRadians(90))
//                .waitSeconds(2)
//
//                // Score Sample #1
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(-135))
//                .waitSeconds(2)
//
//                // Collect Sample #2
//                .strafeToLinearHeading(new Vector2d(-60, -50), Math.toRadians(90))
//                .waitSeconds(2)
//
//                // Score Sample #2
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(-135))
//                .waitSeconds(2)
//
//                // Collect Sample #3
//                .strafeToLinearHeading(new Vector2d(-60, -50), Math.toRadians(110))
//                .waitSeconds(2)
//
//                // Score Sample #3
//                .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(-135))
//                .waitSeconds(2)
//
//                // Park
//                .strafeToLinearHeading(new Vector2d(-24, 0), Math.toRadians(0))
//                .waitSeconds(2)
//
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}