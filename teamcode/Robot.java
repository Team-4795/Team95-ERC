package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot extends OpMode {
    //drive base
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    //conveyor belt arm
    private DcMotor conveyor;
    //Timer maybe not needed
    private final ElapsedTime runtime = new ElapsedTime();

    //PID stuff
    static double distance =50;
    static double sec = 20;

    public static PIDCoefficients PIDCoef = new PIDCoefficients(0,0,0);
    public PIDCoefficients PIDGains = new PIDCoefficients(0,0,0);
    ElapsedTime PIDTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE); //maybe
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

    public void drive(double power, double turn) {
        leftMotor.setPower(power + turn);
        rightMotor.setPower(power - turn);
    }

    public void conStart() {
        conveyor.setPower(.5);
    }

    public void conStop() {
        conveyor.setPower(0);
    }
    //PID

    double integral = 0;
    double LastError = 0;




    public void PID(double targetvelocity)    {
       


            PIDTime.reset();

            double currentVelocity = leftMotor.getPower();

            double error = targetvelocity- currentVelocity;
            integral = integral + error * PIDTime.time();
            double ChangeError = error - LastError;
            double derivative = ChangeError / PIDTime.time();

            PIDGains.p = PIDCoef.p * error;
            PIDGains.i = PIDCoef.i * integral;
            PIDGains.d = PIDCoef.d * derivative;

            leftMotor.setPower(PIDGains.p + PIDGains.i + PIDGains.d + targetvelocity);
            rightMotor.setPower(PIDGains.p + PIDGains.i + PIDGains.d + targetvelocity);
            LastError = error;
        }
    }


}
