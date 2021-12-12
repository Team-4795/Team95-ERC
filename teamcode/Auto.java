package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auto", group = "group1")

public class Auto extends Robot {
    ElapsedTime runtime = new ElapsedTime();

    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
        //if PID works just use:
        PID(20,1);//does pid to move 20 inches during 1 second
        double time=1;
        double t = System.currentTimeMillis();
        double end = t+time;
        while(System.currentTimeMillis() < end) {
            drive(-1,0);
        }
        }




    @Override
    public void loop() {
        conStart();
    }

    @Override
    public void stop() {
    conStop();
    }


}