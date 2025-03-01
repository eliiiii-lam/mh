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

@Autonomous(name = "rubix prime")
public class R_prime extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(0, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armUp())
                .afterTime(0,arm.SetPosition(-475))
                .strafeToLinearHeading(new Vector2d(-24,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))

                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                .setReversed(true)
                /////////////////////////////////////////////////////////////////////////////////////////////
                .splineToLinearHeading(new Pose2d(16.5,30, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(27,8, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(27, 42, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(16,30, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(39,8, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40,42, Math.toRadians(270)), Math.toRadians(270))
             //   .splineToLinearHeading(new Pose2d(25, 35, Math.toRadians(270)), Math.toRadians(270))
               // .splineToLinearHeading(new Pose2d(49,12, Math.toRadians(270)), Math.toRadians(270))
               // .splineToLinearHeading(new Pose2d(49,45, Math.toRadians(270)), Math.toRadians(270))
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,63), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-22,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,65), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-20,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,65), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-18,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .strafeToLinearHeading(new Vector2d(28,65), Math.toRadians(270))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////





                ;














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