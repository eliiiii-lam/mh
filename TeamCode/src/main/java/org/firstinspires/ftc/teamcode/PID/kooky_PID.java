package org.firstinspires.ftc.teamcode.PID;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class kooky_PID extends OpMode {
    private PIDController controller;
    public static double p = 0.0096, i = 0.03, d = 0.000525;
    public static double f = 0.08;
    public static int target;

    private final double ticks_in_degrees = 1425.1/360.0; //change the 360 back to 180 if no work

    private DcMotorEx motor ;

    @Override
    public void init(){

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");


    }


    @Override
    public void loop(){

        controller.setPID(p,i,d);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;



        motor.setPower(power); //setting the motor to the desired position

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();


    }
}