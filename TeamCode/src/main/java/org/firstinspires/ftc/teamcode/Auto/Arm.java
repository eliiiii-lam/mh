package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.Arm_PID_Class;

public class Arm {

    public DcMotorEx Arm;
    public int setPosition;

    Servo linkL;
    Servo linkR;
    Servo inY;
    Servo inX;
    Servo inClaw;
    Servo outRot;
    Servo outClaw;

    Servo inPiv;

    public Arm(HardwareMap hardwareMap) {


        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linkL = hardwareMap.servo.get("linkL");
        linkR = hardwareMap.servo.get("linkR");

        inY = hardwareMap.servo.get("inY");
        inX = hardwareMap.servo.get("inX");
        inClaw = hardwareMap.servo.get("inClaw");
        outRot = hardwareMap.servo.get("outRot");
        outClaw = hardwareMap.servo.get("outClaw");

        inPiv = hardwareMap.servo.get("inPiv");

        linkR.setDirection(Servo.Direction.REVERSE);
        outRot.setDirection(Servo.Direction.REVERSE);


    }

// use public void

    public class updatePID implements Action {
        public updatePID(){

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Arm.setPower(Arm_PID_Class.returnArmPID(setPosition,Arm.getCurrentPosition()));
            return true;
        }
    }
    public Action UpdatePID(){return new updatePID();}

    public class setPosition implements Action {
        int set;
        public setPosition(int position){set = position;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPosition = set;
            return false;
        }
    }
    public Action SetPosition(int pos){return new setPosition(pos);}
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public class armDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outRot.setPosition(0.32);

            return false;
        }
    }
    public Action armDown() {
        return new Arm.armDown();
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public class armUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outRot.setPosition(0.97);
            outClaw.setPosition(0.72);


            return false;
        }
    }
    public Action armUp() {
        return new Arm.armUp();
    }

    //////////////////////////////////////////////////////////////////////////////////////


    public class openClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outClaw.setPosition(0.35);



            return false;
        }
    }
    public Action openClaw() {
        return new Arm.openClaw();
    }

    //////////////////////////////////////////////////////////////////////////////////////
    public class lockIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            linkL.setPosition(0.5);
            linkR.setPosition(0.5);

            inY.setPosition(0.7);//bring claw back
            inPiv.setPosition(0.4);




            return false;
        }
    }
    public Action lockIntake() {
        return new Arm.lockIntake();
    }


    public class closeClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outClaw.setPosition(0.67);



            return false;
        }
    }
    public Action closeClaw() {
        return new Arm.closeClaw();
    }

}