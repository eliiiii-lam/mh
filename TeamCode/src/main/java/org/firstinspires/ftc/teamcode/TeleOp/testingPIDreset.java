
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID.DoubleArrayUtil;

@TeleOp(name = "PID reset", group = "A1")

public class testingPIDreset extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.0096, i = 0.03, d = 0.000525;
    public static double f = 0.08;


    public static int target;
    //public int target;


    private final double down = 0;


    private PIDController controller1;
    public static double p1 = 0.008, i1 = 0, d1 = 0;
    public static double f1 = 0.2;


    public static int target1;

    private final double ticks_in_degrees = 1425.1/360.0; //change the 360 back to 180 if no work

    private DcMotorEx motor ;

    private Motor.Encoder encoder;

    private DcMotorEx uppies ;

    private DoubleArrayUtil window = new DoubleArrayUtil(30);



    boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //testinggggg
        // Declare our motors
        // Make sure your ID's match your configuration

        encoder = new Motor(hardwareMap, "motor", Motor.GoBILDA.RPM_435).encoder;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");

        //Drivetrain drivetrain = new Drivetrain(hardwareMap);
        //drivetrain.setForwardspeed(100);


        Servo linkL = hardwareMap.servo.get("linkL");
        Servo linkR = hardwareMap.servo.get("linkR");


        Servo inY = hardwareMap.servo.get("inY");
        Servo inX = hardwareMap.servo.get("inX");
        Servo inClaw = hardwareMap.servo.get("inClaw");
        Servo inPiv = hardwareMap.servo.get("inPiv");

        //set target to zero here

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller1 = new PIDController(p1,i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "Arm");
        uppies = hardwareMap.get(DcMotorEx.class,"uppies");

        Servo outRot = hardwareMap.servo.get("outRot");
        Servo outClaw = hardwareMap.servo.get("outClaw");

        linkR.setDirection(Servo.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outRot.setDirection(Servo.Direction.REVERSE);




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


            boolean resetRequested = gamepad2.right_stick_button;

            window.populateWindow(encoder.getPosition());
            if (resetRequested) {
                target = 0;
                window.reset();
            }

            if (window.allValuesSame() && window.isWindowFull() && target <= 0) {
                encoder.reset();
                target = 0;
            }



            double triggerVal = gamepad1.left_trigger;
            double powerFactor = slowMode ? 0.35 : 1.0;


            if (triggerVal > 0.1) {
                // Map trigger value from [0.1, 1] to [0.5, 1] for finer control
                powerFactor = 0.35 + 0.35 * (1 - triggerVal);
            }
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                // Map joystick input (-1 to 1) to servo range (0 to 1)
                double servoPosition = (1-(gamepad2.right_stick_x + 1) / 2);

                // Set the servo position
                inX.setPosition(servoPosition);
            } else {
                // Optional: Keep the servo in a neutral position or maintain its current state
                inX.setPosition(0.5); // Set to neutral if joystick is idle
            }








            if (gamepad2.right_bumper){
                outClaw.setPosition(0.35);
                inClaw.setPosition(0.3);
            }

            if (gamepad2.left_bumper) {

                inY.setPosition(0.16);//lower claw to ground-level
                inPiv.setPosition(0.2);
            }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;

            controller1.setPID(p1,i1,d1);
            int armPos1 = uppies.getCurrentPosition();
            double pid1 = controller1.calculate(armPos1, target1);
            double ff1 = Math.cos(Math.toRadians(target1/ticks_in_degrees)) * f1;

            double power1 = pid1 + ff1;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad2.a) {
                inPiv.setPosition(0.15);
                inY.setPosition(0.25);//lower claw to "observe mode"
                linkL.setPosition(0.25);
                linkR.setPosition(0.25);
            }

            if (gamepad2.b){
                inY.setPosition(0.7);//bring claw back
                inPiv.setPosition(0.4);
                linkL.setPosition(0.56);
                linkR.setPosition(0.56);
            }

            if (gamepad2.x){
                outRot.setPosition(0.32);

                target1 = 1020;
                target = -1100;
                outClaw.setPosition(0.5);
                inY.setPosition(0.5);//bring claw back
                inPiv.setPosition(0);
                linkL.setPosition(0.53);
                linkR.setPosition(0.53);
                inClaw.setPosition(0.51);//close claw
            }

            if (gamepad2.y){
                inClaw.setPosition(0.4);
                outClaw.setPosition(0.65);
                sleep(100);
                outRot.setPosition(0.32);

                target1 = 2300;
                target = -400;
            }


            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            if (gamepad2.right_stick_button){
               // target = armPos + target;

            }


            if (gamepad2.left_stick_button){
                inClaw.setPosition(0.51);//close claw
                //target = target + 2;
            }


            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad2.dpad_down){
                outRot.setPosition(0.32);
                target = 0;
                target1 = -75;

            }


            if (gamepad2.dpad_left){
                outClaw.setPosition(0.68);
                sleep(450);
                target = -475;
                outRot.setPosition(0.97);
            }

            if (gamepad2.dpad_up){
                outClaw.setPosition(0.68);
                target = -850; // changed
                outRot.setPosition(0.97);
            }

            if (gamepad2.dpad_right){
                outRot.setPosition(0.32);

                outClaw.setPosition(0.68);
                target = -750; // changed
            }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





            //main code here
            motor.setPower(power); //setting the motor to the desired position
            uppies.setPower(power1); //setting the motor to the desired position


            telemetry.addData("pos ", armPos);
            telemetry.addData("target ", target);
            telemetry.addData("slides pos ", armPos1);
            telemetry.addData("slides target ", target1);
            telemetry.update();

            double y = gamepad1.left_stick_y *  1; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x  * 1;
            double rx = -gamepad1.right_stick_x * 1;
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