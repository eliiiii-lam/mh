/* Copyright (c) 2022 FIRST. All rights reserved. */
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@Autonomous(name="IMU", group="Auto")
public class IMU_drive extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    Servo link;
    Servo inY;
    Servo inX;
    Servo inClaw;
    Servo inmainPiv;
    Servo outY;
    Servo outRot;
    Servo outClaw;
    Servo outmainPiv;
    Servo outmainPiv2;


    private IMU imu = null;      // Control/Expansion Hub IMU

    private double headingError = 0;

    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    private int     flTarget = 0;
    private int     frTarget = 0;
    private int     blTarget = 0;
    private int     brTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0 ;    // How close must the heading get to the target before moving to next step.
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.


    @Override
    public void runOpMode() {

        // Initialize the four drive motors.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");


        link = hardwareMap.servo.get("link");
        inY = hardwareMap.servo.get("inY");
        inX = hardwareMap.servo.get("inX");
        inClaw = hardwareMap.servo.get("inClaw");
        inmainPiv = hardwareMap.servo.get("inmainPiv");
        outY = hardwareMap.servo.get("outY");
        outRot = hardwareMap.servo.get("outRot");
        outClaw = hardwareMap.servo.get("outClaw");
        outClaw = hardwareMap.servo.get("outClaw");
        outmainPiv = hardwareMap.servo.get("outmainPiv");
        outmainPiv2 = hardwareMap.servo.get("outmainPiv2");


        // Reverse the right motors (fR and bR)
        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        link.setDirection(Servo.Direction.REVERSE);
        outmainPiv2.setDirection(Servo.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize IMU with the given orientation.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary. Reset encoders and set to BRAKE mode
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();

            if (gamepad2.y){
                imu.resetYaw();
            }
            ////////////////////////////////
            outmainPiv.setPosition(0.1);
            outmainPiv2.setPosition(0.03);
            outY.setPosition(1);
            outRot.setPosition(0.27);
            outClaw.setPosition(0.96);
            ////////////////////////////////

        }

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
waitForStart();
        link.setPosition(0.62); // bring intake back
        inY.setPosition(0.55);//bring claw back
        inClaw.setPosition(0.43);//open claw
        outmainPiv.setPosition(0.52);
        outmainPiv2.setPosition(0.5);
        outY.setPosition(0.5);
        outRot.setPosition(0.95);
        driveStraight(0.2, -60, 0.0);    // drive to bar
        //arm func


        outmainPiv.setPosition(1);
        outmainPiv2.setPosition(1);
        outY.setPosition(0.5);
        outRot.setPosition(0.95);
        outY.setPosition(0.5);
        sleep(1000);
        outClaw.setPosition(0.5);
        //arm func
        sleep(500);

        driveStraight(DRIVE_SPEED, 50, 0.0);    // drive to bar
////////////////////////////////
        outmainPiv.setPosition(0.12);
        outmainPiv2.setPosition(0.09);
        outY.setPosition(0.71);
        outRot.setPosition(0.27);
        outClaw.setPosition(0.5);
        //////////////////////////////////
turnToHeading(0.4,90);
        outmainPiv.setPosition(0.19);
        outmainPiv2.setPosition(0.14);
        outY.setPosition(0.71);
        outRot.setPosition(0.27);
        outClaw.setPosition(0.5);
/////////////////////////////////////////
        driveStraight(DRIVE_SPEED,62,90);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    // High-level driving functions

    public void strafeR(double maxDriveSpeed,
                        double distance,
                        double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            flTarget = bL.getCurrentPosition() + moveCounts;
            frTarget = fR.getCurrentPosition() - moveCounts;
            blTarget = bL.getCurrentPosition() - moveCounts;
            brTarget = bR.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            fR.setTargetPosition(frTarget);
            fL.setTargetPosition(blTarget);
            bR.setTargetPosition(brTarget);
            bL.setTargetPosition(blTarget);

            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (fR.isBusy() && (fL.isBusy() && (bR.isBusy() && (bL.isBusy()))))) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(false);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void strafeL(double maxDriveSpeed,
                        double distance,
                        double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            flTarget = bL.getCurrentPosition() - moveCounts;
            frTarget = fR.getCurrentPosition() + moveCounts;
            blTarget = bL.getCurrentPosition() + moveCounts;
            brTarget = bR.getCurrentPosition() - moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            fR.setTargetPosition(frTarget);
            fL.setTargetPosition(blTarget);
            bR.setTargetPosition(brTarget);
            bL.setTargetPosition(blTarget);

            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (fR.isBusy() && (fL.isBusy() && (bR.isBusy() && (bL.isBusy()))))) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(false);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = fL.getCurrentPosition() + moveCounts;
            rightTarget = fR.getCurrentPosition() + moveCounts;

            fL.setTargetPosition(leftTarget);
            bL.setTargetPosition(leftTarget);
            fR.setTargetPosition(rightTarget);
            bR.setTargetPosition(rightTarget);

            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() && (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);
                sendTelemetry(true);
            }

            moveRobot(0, 0);
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        fL.setPower(leftSpeed);
        bL.setPower(leftSpeed);
        fR.setPower(rightSpeed);
        bR.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", fL.getCurrentPosition(), fR.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}