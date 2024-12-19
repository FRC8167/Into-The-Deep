package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineTests {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 63.5, Math.toRadians(-90)))
//                .strafeTo(new Vector2d(-20,55))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-43, 30, Math.toRadians(180)), Math.toRadians(180))
//                .build());
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 55, Math.toRadians(-90)))
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(-48, 24, Math.toRadians(-90)), Math.toRadians(180))
                .build());
        //1) bot heading at start
        //2)  start direction of spline
        //3) bot heading at end
        //4)  direction of end of spline

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}