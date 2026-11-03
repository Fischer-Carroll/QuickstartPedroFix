package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Drive Testing")
public class Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, gamepad1,telemetry);
        bot.initialiseServoScales();
        waitForStart();
        while (opModeIsActive()){
            bot.handleDepollinator();
        }
    }
}
