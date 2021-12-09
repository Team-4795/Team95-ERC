package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class PIDAuto extends BasicOpMode_Linear{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    static double distance =50;
    static double sec = 20;

    public static PIDCoefficients PIDCoef = new PIDCoefficients(0,0,0);
    public PIDCoefficients PIDGains = new PIDCoefficients(0,0,0);
    ElapsedTime PIDTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpmode()
    {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if(opModeIsActive())
        {
            while(opModeIsActive())
            {
                PID(distance,sec);

                telemetry.update();
            }
        }

    }
    double integral = 0;
    double LastError = 0;




public void PID(double targetDistance,double time)    {
    double t = System.currentTimeMillis();
    double end = t+time;
    double InPerTick = .01;
    double targetvelocity = targetDistance/InPerTick/time;
    while(System.currentTimeMillis() < end) {


        PIDTime.reset();

        double currentVelocity = leftDrive.getPower();

        double error = targetDistance - InPerTick * currentVelocity;
        integral = integral + error * PIDTime.time();
        double ChangeError = error - LastError;
        double derivative = ChangeError / PIDTime.time();

        PIDGains.p = PIDCoef.p * error;
        PIDGains.i = PIDCoef.i * integral;
        PIDGains.d = PIDCoef.d * derivative;

        leftDrive.setPower(PIDGains.p + PIDGains.i + PIDGains.d + targetvelocity);
        LastError = error;
    }
}
}

