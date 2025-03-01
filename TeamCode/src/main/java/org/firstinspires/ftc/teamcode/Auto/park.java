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

@Autonomous(name = "park")
public class park extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for R
        Pose2d initialPose = new Pose2d(0, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(40,60), Math.toRadians(270));




















        Action TrajectoryClose = spec1.endTrajectory().build();


        if (isStopRequested()) return;




        Actions.runBlocking(


                new ParallelAction(
                        spec1.build()

                )


        );
    }
}