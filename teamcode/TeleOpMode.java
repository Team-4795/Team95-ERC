package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="main teleop", group = "group1")
public class TeleOpMode extends Robot {
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
    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.right_bumper) {
            conStart();
        } else if (gamepad1.left_bumper) {
            conStop();
        }
    }

    @Override
    public void stop() {

    }
}