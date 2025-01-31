package com.jedisonknights.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(
                new Pose2d(-10, -61, Math.PI/2))
                // Drop off preload
                .splineTo(new Pose2d(-10, -34, Math.PI/2).component1(), Math.PI/2)
                                .waitSeconds(4)

                        .lineToY(-40)/*
                .strafeTo(new Vector2d(-35, -40))
                .strafeTo(new Vector2d(-35, -0))
                .turn(-Math.PI/2)
                .lineToX(-24) */


                 // First sample pickup
                    .splineTo(new Vector2d(-52, -60), Math.PI)
                        .turn(Math.PI+20*Math.PI/180)
                .waitSeconds(1)

                //.turn(Math.PI/2)
                        .turn(-(Math.PI/2 + (double) 5 /180*Math.PI +20*Math.PI/180))
                        .waitSeconds(1)
                        .turn(Math.PI/2 + (double) 5 /180*Math.PI +20*Math.PI/180)
                        .waitSeconds(1)
                   // .strafeTo(new Vector2d(-52, -60))

                .waitSeconds(1)



                // Second sample pickup

                    .turn(-80*Math.PI/180 -20*Math.PI/180)
                        .waitSeconds(1)
                .turn(80*Math.PI/180 + +20*Math.PI/180)
                .waitSeconds(1)


                // Third sample pickup
                        .turn(120*Math.PI/180-Math.PI-20*Math.PI/180)
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(-52, -60))
                .turn(-1*(120*Math.PI/180-Math.PI-20*Math.PI/180))
                        .waitSeconds(1)

                        .strafeTo(new Vector2d(-52, -40))

                        .splineTo(new Vector2d(-24, -12), -Math.PI/2 - 20*Math.PI/180)



                .build());

                /*
                new Pose2d(8, 61, -Math.PI/2))

                // Specimen Dropoff
                                .lineToY(35)
                                .waitSeconds(4)


                // First sample pickup
                    .strafeTo(new Vector2d(48, 37))
                    .waitSeconds(1)


                /*
                // Second sample pickup
                    .strafeTo(new Vector2d(58, 35))
                    .turn(-Math.PI/2)
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(52, 60))
                    .turn(Math.PI/2)
                    .waitSeconds(1)

                // Third sample pickup
                    .strafeTo(new Vector2d(55, 28))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(52, 60))
                    .waitSeconds(1)

                    .strafeTo(new Vector2d(52, 40))

                    .splineTo(new Vector2d(24, 12), 0)
                    .turn(Math.PI/2)

                .build());*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}