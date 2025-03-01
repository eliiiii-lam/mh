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

@Autonomous(name = "rubix alt")
public class rubix extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for R
        Pose2d initialPose = new Pose2d(0, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .stopAndAdd(arm.armUp())
                .afterTime(0.1,arm.SetPosition(-125))
                .splineToLinearHeading(new Pose2d(-8,40, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-350)) //slam spec
                .waitSeconds(0.5)
                .stopAndAdd(arm.openClaw())
                .strafeToLinearHeading(new Vector2d(22,50), Math.toRadians(270))
               .afterTime(0, arm.SetPosition(400))
                .stopAndAdd(arm.armDown())
                .strafeToLinearHeading(new Vector2d(22,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(30,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(30,60), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(30,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40,10), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40,60), Math.toRadians(270))
                .waitSeconds(0.2)
                .stopAndAdd(arm.closeClaw())
                .afterTime(0.5,arm.SetPosition(-125))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-10,34), Math.toRadians(270))
                .afterTime(0.3,arm.SetPosition(-350)) //slam spec
                .waitSeconds(0.5)
                .stopAndAdd(arm.openClaw())
                .afterTime(0, arm.SetPosition(400))
                .stopAndAdd(arm.armDown())
                .strafeToLinearHeading(new Vector2d(35,60), Math.toRadians(270))
                .waitSeconds(0)
                .stopAndAdd(arm.closeClaw())
                .afterTime(0.5,arm.SetPosition(-125))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-12,34), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-350)) //slam spec
                .waitSeconds(0.5)
                .stopAndAdd(arm.openClaw())
                .afterTime(0, arm.SetPosition(400))
                .stopAndAdd(arm.armDown())
                .strafeToLinearHeading(new Vector2d(35,60), Math.toRadians(270))
                .waitSeconds(0)
                .stopAndAdd(arm.closeClaw())
                .afterTime(0.5,arm.SetPosition(-125))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-14,34), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-350)) //slam spec
                .waitSeconds(0.5)
                .stopAndAdd(arm.openClaw())
                .afterTime(0, arm.SetPosition(-350))
                .stopAndAdd(arm.armDown());




















        Action TrajectoryClose = spec1.endTrajectory().build();


        if (isStopRequested()) return;




        Actions.runBlocking(


                new ParallelAction(
                        arm.lockIntake(),
                        spec1.build(),
                        arm.UpdatePID()

                )


        );
    }
}