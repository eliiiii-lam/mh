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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "rubix 5")
public class R_five_spec extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(-5, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "Arm");

        if (gamepad2.right_stick_button){

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.closeClaw())
                .stopAndAdd(arm.armUp())
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.closeClaw())
                .lineToY(30)
                .afterTime(0,arm.SetPosition(-800))

                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.2,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                .setReversed(true)
                /////////////////////////////////////////////////////////////////////////////////////////////
                .splineToLinearHeading(new Pose2d(20,30, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(25,15, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(35,12, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(35, 53, Math.toRadians(270)), Math.toRadians(270))
                /////////////////////////////////////////////////////////////////////////////////////////////
                .splineToLinearHeading(new Pose2d(25,15, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46,14, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46,53, Math.toRadians(270)), Math.toRadians(270))
                /////////////////////////////////////////////////////////////////////////////////////////////
                .splineToLinearHeading(new Pose2d(30, 35, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, 15, Math.toRadians(270)), Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(58,14, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 0.6))
                .splineToLinearHeading(new Pose2d(60,50, Math.toRadians(260)), Math.toRadians(260))
                /////////////////////////////////////////////////////////////////////////////
                //.splineToLinearHeading(new Pose2d(35,50, Math.toRadians(270)), Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(28,61, Math.toRadians(270)), Math.toRadians(270))
                .stopAndAdd(arm.closeClaw())
                ////////
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-28,29), Math.toRadians(265), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(28,61), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-26,30), Math.toRadians(265), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())
                /////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(31,62), Math.toRadians(265), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-22,32), Math.toRadians(265), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 3.9))
                .afterTime(0,arm.SetPosition(-800))
                .waitSeconds(0.1)
                .stopAndAdd(arm.openClaw())
                .afterTime(0.3,arm.SetPosition(0))
                .stopAndAdd(arm.armDown())

                .strafeToLinearHeading(new Vector2d(31,62), Math.toRadians(265), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 3.9))
                /////////////////////////////////////////////////////////////////////////////
                .stopAndAdd(arm.closeClaw())
                .waitSeconds(0.15)
                .afterTime(0,arm.SetPosition(-475))
                .stopAndAdd(arm.armUp())
                .strafeToLinearHeading(new Vector2d(-22,33), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 4.5))
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