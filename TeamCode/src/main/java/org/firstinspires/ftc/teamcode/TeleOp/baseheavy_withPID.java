
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "Baseheavy PID", group = "TELE")

public class baseheavy_withPID extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.008, i = 0.01, d = 0.001;
    public static double f = 0.08;
    public static int target = 0;

    private final double ticks_in_degrees = 1425.1/360.0; //change the 360 back to 180 if no work

    private DcMotorEx motor ;


    boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //testinggggg
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");

        DcMotor uppies = hardwareMap.dcMotor.get("uppies");

        Servo link = hardwareMap.servo.get("link");

        Servo inY = hardwareMap.servo.get("inY");
        Servo inX = hardwareMap.servo.get("inX");
        Servo inClaw = hardwareMap.servo.get("inClaw");
        Servo inmainPiv = hardwareMap.servo.get("inmainPiv");


        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        Servo outY = hardwareMap.servo.get("outY");
        Servo outRot = hardwareMap.servo.get("outRot");
        Servo outClaw = hardwareMap.servo.get("outClaw");

        link.setDirection(Servo.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double triggerVal = gamepad1.left_trigger;
            double powerFactor = slowMode ? 0.35 : 1.0;


            if (triggerVal > 0.1) {
                // Map trigger value from [0.1, 1] to [0.5, 1] for finer control
                powerFactor = 0.35 + 0.35 * (1 - triggerVal);
            }


            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;




            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (Math.abs(gamepad2.left_stick_y) > 0.1){
                uppies.setPower(gamepad2.left_stick_y * 0.8);
            } else {
                uppies.setPower(0);
                uppies.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                // Map joystick input (-1 to 1) to servo range (0 to 1)
                double servoPosition = (1-(gamepad2.left_stick_x + 1) / 2);

                // Set the servo position
                inX.setPosition(servoPosition);
            } else {
                // Optional: Keep the servo in a neutral position or maintain its current state
                inX.setPosition(0.5); // Set to neutral if joystick is idle
            }



            if (gamepad2.a) {
                inmainPiv.setPosition(0.5);
                link.setPosition(0.23);
                inY.setPosition(0.45);//lower claw to "observe mode"
            }

            if (gamepad2.b){
                link.setPosition(0.62); // bring intake back
                inY.setPosition(0.55);//bring claw back
            }

            if (gamepad2.x){
                link.setPosition(0.397); // Set to neutral if joystick is idle
                sleep(200);
                inmainPiv.setPosition(0.5);
                sleep(100);
                inY.setPosition(0.95);//bring claw back
                sleep(500);
                outClaw.setPosition(0.5);

                outY.setPosition(0.93);
                outRot.setPosition(0.95);

            }

            if (gamepad2.y){
                outClaw.setPosition(0.96);
                inClaw.setPosition(0.43);//open claw
                sleep(300);
                inClaw.setPosition(0.43);//open claw

                outY.setPosition(0.5);
                outRot.setPosition(0.95);


            }


            if (gamepad2.left_bumper) {

                inY.setPosition(0.36);//lower claw to ground-level
                sleep(400);
                inClaw.setPosition(0.57);//close claw
            }

            if (gamepad2.right_bumper){
                inClaw.setPosition(0.43);//open claw
                outClaw.setPosition(0.5);
            }


            if (gamepad1.a){
                outY.setPosition(0.55);
                outRot.setPosition(0.27);
                target = -100;
            }

            if (gamepad1.x){
                outClaw.setPosition(0.7);
                sleep(500);
                target = -650;
                outY.setPosition(0.55);
                outRot.setPosition(0.95);
            }

            if (gamepad1.y){
                target = -800;
                outY.setPosition(0.55);
                outRot.setPosition(0.95);
                sleep(500);
            }

            if (gamepad1.right_bumper){
                outClaw.setPosition(0.5);
            }

            //main code here
            motor.setPower(power); //setting the motor to the desired position

            telemetry.addData("pos ", armPos);
            telemetry.addData("target ", target);
            telemetry.update();

            double y = gamepad1.left_stick_y *  0.8; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x  * 0.8;
            double rx = -gamepad1.right_stick_x * 0.8;
            y *= powerFactor;
            x *= powerFactor;
            rx *= powerFactor;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        }
    }
}