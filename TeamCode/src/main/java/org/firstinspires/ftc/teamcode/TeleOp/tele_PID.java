package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled

@Config
@TeleOp(name = "tele with PID", group = "TELE")
public class tele_PID extends OpMode {

    private ElapsedTime timer = new ElapsedTime();
    private boolean isWaiting = false;

    private PIDController controller;
    public static double p = 0.2, i = 0.2, d = 0.00001;
    public static double f = 0.7;
    public static int target = 0;

    private final double ticks_in_degrees = 384.5 / 360.0;
    private DcMotorEx motor;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, uppies;

    Servo link;
    Servo inY;
    Servo inX;
    Servo inClaw;
    Servo inmainPiv;
    Servo outY;
    Servo outRot;
    Servo outClaw;

    private boolean slowMode = false;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        backRightMotor = hardwareMap.dcMotor.get("bR");

        uppies = hardwareMap.dcMotor.get("uppies");
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        Servo link = hardwareMap.servo.get("link");

        link = hardwareMap.servo.get("link");
        inY = hardwareMap.servo.get("inY");
        inX = hardwareMap.servo.get("inX");
        inClaw = hardwareMap.servo.get("inClaw");
        inmainPiv = hardwareMap.servo.get("inmainPiv");
        outY = hardwareMap.servo.get("outY");
        outRot = hardwareMap.servo.get("outRot");
        outClaw = hardwareMap.servo.get("outClaw");
        outClaw = hardwareMap.servo.get("outClaw");





        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        link.setDirection(Servo.Direction.REVERSE);

        uppies.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }




    @Override
    public void loop() {

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        controller.setPID(p, i, d);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;

        motor.setPower(power);

        double triggerVal = gamepad1.left_trigger;
        double powerFactor = slowMode ? 0.35 : 1.0;
        if (triggerVal > 0.1) {
            powerFactor = 0.35 + 0.35 * (1 - triggerVal);
        }

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        ////////////////////////////////////////////////////////////////////////////////////
        if (Math.abs(gamepad2.left_stick_y) > 0.1){
            uppies.setPower(gamepad2.left_stick_y * 0.8);
        } else {
            uppies.setPower(0);
            uppies.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.1){
            motor.setPower(gamepad2.right_stick_y * 0.8);
        } else {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
          //  sleep(200);
            inmainPiv.setPosition(0.5);
          // sleep(100);
            inY.setPosition(0.95);//bring claw back
          //  sleep(500);
            outClaw.setPosition(0.5);

            outY.setPosition(0.93);
            outRot.setPosition(0.95);

        }

        if (gamepad2.y){
            outClaw.setPosition(0.96);
            inClaw.setPosition(0.43);//open claw
           // sleep(300);
            inClaw.setPosition(0.43);//open claw

            outY.setPosition(0.5);
            outRot.setPosition(0.95);


        }


        if (gamepad2.left_bumper) {

            inY.setPosition(0.36);//lower claw to ground-level
           // sleep(400);
            inClaw.setPosition(0.57);//close claw
        }

        if (gamepad2.right_bumper){
            inClaw.setPosition(0.43);//open claw
            outClaw.setPosition(0.5);
        }


        if (gamepad2.dpad_down){

            // outY.setPosition(0.71);
            // outRot.setPosition(0.27);
        }
        if (gamepad2.dpad_left){
            //  outClaw.setPosition(0.96);
            //sleep(300);

            // outY.setPosition(0.5);
            // outRot.setPosition(0.95);

        }
        if (gamepad2.dpad_up){

            //  outY.setPosition(0.5);
            // outRot.setPosition(0.95);
            //sleep(700);
            // outClaw.setPosition(0.5);//dont close claw


        }

        if (gamepad2.dpad_right){
            //outClaw.setPosition(0.96);

            outY.setPosition(0.5);
            // outRot.setPosition(0.95);
            // sleep(700);


        }
        /////////////////////////////////////////////////////////////////////////////

        double y = gamepad1.left_stick_y * 0.8;
        double x = -gamepad1.left_stick_x * 0.8;
        double rx = -gamepad1.right_stick_x * 0.8;

        y *= powerFactor;
        x *= powerFactor;
        rx *= powerFactor;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
