package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAuto1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -63.5, Math.toRadians(90)))
                .waitSeconds(2)
                .strafeTo(new Vector2d(0,-33.5))
                .strafeTo(new Vector2d(20,-55))
                .setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(43, -30), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(46, -30), Math.toRadians(-10))
                .strafeToSplineHeading(new Vector2d(55, -62), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(55, -30), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(55, -47.5), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(0,-50), Math.toRadians(90))
                .strafeTo(new Vector2d(0,-33.5))
                .strafeTo(new Vector2d(0,-55))
                .strafeToSplineHeading(new Vector2d(55, -60), Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}