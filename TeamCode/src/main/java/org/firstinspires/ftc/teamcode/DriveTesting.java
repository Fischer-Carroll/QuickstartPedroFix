package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Drive Testing")
public class DriveTesting extends OpMode {

    private Follower follower;

    // Ignore tiny right-stick movements
    private double stickDeadzone = 0.12;
    Servo turret1;
    Servo turret2;
    double turretPos;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        turretPos = 0.5;
        follower.setPose(new Pose(0,0,Math.toRadians(90)));
        turret1 = hardwareMap.get(Servo.class, "turretLat1");
        turret2 = hardwareMap.get(Servo.class, "turretLat2");
        turret2.setPosition(turretPos);
        turret1.setPosition(turretPos);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()){
            turretPos += 0.1;
        }
        if (gamepad1.dpadRightWasPressed()){
            turretPos -= 0.1;
        }
        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos)
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );
        
        telemetry.addData("Turret1 Position", turret1.getPosition());
        telemetry.addData("Turret2 Position", turret2.getPosition());
        telemetry.addData("Passed turret Pos", turretPos);
        telemetry.update();
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }


}
