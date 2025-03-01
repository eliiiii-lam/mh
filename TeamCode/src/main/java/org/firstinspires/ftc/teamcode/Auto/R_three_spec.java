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

@Autonomous(name = "rubix 3 spec")
public class R_three_spec extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(0, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .stopAndAdd(arm.armDown())
                .stopAndAdd(arm.openClaw())
                .afterTime(0,arm.SetPosition(0))
                .setReversed(false)
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
                .strafeToLinearHeading(new Vector2d(23,63), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-28,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(25,63), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-26,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(25,63), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.3)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-24,32), Math.toRadians(270))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.8,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(25,63), Math.toRadians(270))
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
                //.strafeToLinearHeading(new Vector2d(20,61.5), Math.toRadians(270))
                // .strafeToLinearHeading(new Vector2d(-22,35), Math.toRadians(270))
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