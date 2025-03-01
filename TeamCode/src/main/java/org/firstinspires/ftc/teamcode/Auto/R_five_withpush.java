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

@Autonomous(name = "rubix push")
public class R_five_withpush extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(-5, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armUp())
                .afterTime(0,arm.SetPosition(-475))
                .lineToY(32)
                .afterTime(0,arm.SetPosition(-800))

                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.2,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                .setReversed(true)
                /////////////////////////////////////////////////////////////////////////////////////////////
                .splineToLinearHeading(new Pose2d(30,35, Math.toRadians(270)), Math.toRadians(270))

                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(30,20), Math.toRadians(330), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(30,40), Math.toRadians(360), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(40,20), Math.toRadians(330), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(40,40), Math.toRadians(360), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(50,20), Math.toRadians(330), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .strafeToLinearHeading(new Vector2d(50,40), Math.toRadians(360), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))


                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))


                .stopAndAdd(arm.closeClaw())
                ////////
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-16,36), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-14,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.5,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-12,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())

                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 3.9))
                /////////////////////////////////////////////////////////////////////////////
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-10,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 4.5))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 4.5))
                .stopAndAdd(arm.armDown())




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