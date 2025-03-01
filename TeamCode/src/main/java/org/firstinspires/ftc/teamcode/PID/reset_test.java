package org.firstinspires.ftc.teamcode.PID;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PID.DoubleArrayUtil;

@TeleOp
public class reset_test extends OpMode {
    private Motor.Encoder encoder;
    private DcMotorSimple motor;
    private PIDController controller = new PIDController(0.0096,0.03,0.000525);
    private double target_ticks;
    private double upPos_ticks = -500;
    private final double RESET_POS_TICKS = 0;
    private DoubleArrayUtil window = new DoubleArrayUtil(30);
    @Override
    public void init() {
        encoder = new Motor(hardwareMap, "Arm", Motor.GoBILDA.RPM_117).encoder;
        motor = hardwareMap.get(DcMotorSimple.class, "Arm");
    }

    @Override
    public void loop() {
        boolean resetRequested = gamepad1.a;
        window.populateWindow(encoder.getPosition());
        if (resetRequested) {
            target_ticks = RESET_POS_TICKS;
            window.reset();
        }
        if (gamepad1.b) target_ticks = upPos_ticks;
        if (window.allValuesSame() && window.isWindowFull() && target_ticks <= 0) {
            encoder.reset();
            target_ticks = 0;
        }
        motor.setPower(controller.calculate(encoder.getPosition(), target_ticks));
    }
}