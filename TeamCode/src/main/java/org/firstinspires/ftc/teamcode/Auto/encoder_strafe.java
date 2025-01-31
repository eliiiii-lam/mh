
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 */
@Disabled
@Autonomous(name="enco strafe", group="Auto")
public class encoder_strafe extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor fL = null; // Front Left motor
    private DcMotor fR = null; // Front Right motor
    private DcMotor bL = null; // Back Left motor
    private DcMotor bR = null; // Back Right motor

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

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5; // TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
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
        // To drive forward, motors on opposite sides should run in reverse direction
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);


        link.setDirection(Servo.Direction.REVERSE);
        outmainPiv2.setDirection(Servo.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeInInit()) {

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

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        strafe(0.4,20,-20,-20,20,2.0);//strafe R
        // Step through each leg of the path
        link.setPosition(0.62); // bring intake back
        inY.setPosition(0.55);//bring claw back
        inClaw.setPosition(0.43);//open claw
        outmainPiv.setPosition(0.52);
        outmainPiv2.setPosition(0.5);
        outY.setPosition(0.5);
        outRot.setPosition(0.95);
        encoderDrive(0.2,70,70,3.5);


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
        encoderDrive(0.4,-40,-40,1.0);
        ////////////////////////////////
        outmainPiv.setPosition(0.12);
        outmainPiv2.setPosition(0.09);
        outY.setPosition(0.71);
        outRot.setPosition(0.27);
        outClaw.setPosition(0.5);
        //////////////////////////////////
        encoderDrive(0.4,-29,29,2);
        strafe(0.4,-20,20,20,-20,2.0);//strafe R
        outmainPiv.setPosition(0.19);
        outmainPiv2.setPosition(0.14);
        outY.setPosition(0.71);
        outRot.setPosition(0.27);
        outClaw.setPosition(0.5);
        encoderDrive(0.4,-62,-62,3.0);

        sleep(500);
        outClaw.setPosition(0.96);//close claw on specimen
        sleep(1000);
        outmainPiv.setPosition(0.52);
        outmainPiv2.setPosition(0.5);
        encoderDrive(DRIVE_SPEED,50,50,2.5);
        outmainPiv.setPosition(0.52);
        outmainPiv2.setPosition(0.5);
        outY.setPosition(0.5);
        outRot.setPosition(0.95);
        strafe(0.4,10,-10,-10,10,1.0);
        strafe(0.4,20,-20,20,-20,2.0);
        encoderDrive(0.2,70,70,3);
        ///////////////////////////////
        outmainPiv.setPosition(1);
        outmainPiv2.setPosition(1);
        outY.setPosition(0.55);
        outRot.setPosition(0.95);
        outY.setPosition(0.48);
        sleep(1000);
        outClaw.setPosition(0.5);
        sleep(1000);


        outmainPiv.setPosition(0.19);
        outmainPiv2.setPosition(0.14);
        outY.setPosition(0.71);
        outRot.setPosition(0.27);
        outClaw.setPosition(0.5);
        encoderDrive(0.4,-62,-62,3.0);
        encoderDrive(0.4,-29,29,2);
        strafe(0.4,-20,20,20,-20,2.0);//strafe R
        encoderDrive(0.8,-62,-62,3.0);
        /////////////////////////////////

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     * Method to perform a relative move, based on encoder counts.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the OpMode running.
     */

    public void strafe(double speed, double frontLeft,
                       double frontRight, double backLeft,
                       double backRight, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() + (int) (frontRight * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() + (int) (backLeft * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderDrive(double speed,
                             double leftDrive, double rightDrive, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();
            }


            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }
}