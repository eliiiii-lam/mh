package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "rubix")
public class rubix extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(0, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        resetRuntime();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-8,34, Math.toRadians(270)), Math.toRadians(270));
        TrajectoryActionBuilder spec2 =  spec1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(22,40), Math.toRadians(270));//allign with first sample
        TrajectoryActionBuilder spec3 =  spec2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(22,10), Math.toRadians(270));//drop first sample in wing TrajectoryActionBuilder spec4 =  spec3.endTrajectory().fresh()
        TrajectoryActionBuilder spec4 =  spec3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30,10), Math.toRadians(270));//drop second sample in wing
        TrajectoryActionBuilder spec5 =  spec4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30,60), Math.toRadians(270));

        TrajectoryActionBuilder spec6 =  spec5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30,10), Math.toRadians(270));

        TrajectoryActionBuilder spec7 =  spec6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,10), Math.toRadians(270));

        TrajectoryActionBuilder spec8 =  spec7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,60), Math.toRadians(270));
////////////////////////////////////////////////////////////////////////////////////////////
        TrajectoryActionBuilder spec9 =  spec8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8,34), Math.toRadians(270));

        TrajectoryActionBuilder spec10 =  spec9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30,60), Math.toRadians(270));

        TrajectoryActionBuilder spec11 =  spec10.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8,34), Math.toRadians(270));

        TrajectoryActionBuilder spec12 =  spec11.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30,60), Math.toRadians(270));

        TrajectoryActionBuilder spec13 =  spec12.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8,34), Math.toRadians(270));



        Action TrajectoryClose = spec1.endTrajectory().build();


        if (isStopRequested()) return;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        Action fwgo1;
        fwgo1 = spec1.build();
        Action fwgo2;
        fwgo2 = spec2.build();
        Action fwgo3;
        fwgo3 = spec3.build();
    Action fwgo4;
            fwgo4 = spec4.build();
        Action fwgo5;
            fwgo5 = spec5.build();
        Action fwgo6;
        fwgo6 = spec6.build();
        Action fwgo7;
        fwgo7 = spec7.build();
        Action fwgo8;
        fwgo8 = spec8.build();

        Action fwgo9;
        fwgo9 = spec9.build();
        Action fwgo10;
        fwgo10 = spec10.build();
        Action fwgo11;
        fwgo11 = spec11.build();
        Action fwgo12;
        fwgo12 = spec12.build();

        Action fwgo13;
        fwgo13 = spec13.build();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                fwgo1

                        ),
                        new SleepAction(0.5),
                        new SequentialAction(

                                fwgo1
                        ),

                        new SequentialAction(
                                fwgo2
                        ),
                        new SequentialAction(
                                fwgo3
                        ),
                        new SequentialAction(
                                fwgo4
                        ),
                        new SequentialAction(
                                fwgo5
                        ), new SequentialAction(
                        fwgo6
                ), new SequentialAction(
                        fwgo7
                ), new SequentialAction(
                        fwgo8
                ), new SequentialAction(
                        fwgo9
                ), new SequentialAction(
                        fwgo10
                ), new SequentialAction(
                        fwgo11
                ), new SequentialAction(
                        fwgo12
                ), new SequentialAction(
                        fwgo13
                )

                )
        );
    }
}