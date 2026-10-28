package org.firstinspires.ftc.teamcode;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public class Calibration {
    Follower follower;
    double x;
    double y;
    double w = 7.75;
    double l = 9;
    double heading;
    double targetHive;
    double distance;
    double flywheelkP;
    double flywheelkF;
    double flywheelVelocity;
    double hoodAngle;
    double turretAngle;
    double depollinatorAngle;
    double intakePower;
    double transferPower;

    //Initialize passed values
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    //Initialise things from hardware map 
    DcMotorEx bottomMotor;
    DcMotorEx topMotor;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    Servo turret1;
    Servo turret2;
    Servo blocker;
    Servo depollinator;
    Servo hood;

    public Calibration(HardwareMap hwMap, Gamepad gp1, Gamepad gp2, Telemetry tlm) {
        //Declare passed objects
        hardwareMap = hwMap;
        gamepad1 = gp1;
        gamepad2 = gp2;
        telemetry = tlm;

        //Declare hardware map
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depollinator = hardwareMap.get(Servo.class, "depollinator");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        //Assign limits to servos
        blocker.scaleRange(0, 1);
        depollinator.scaleRange(0, 1);
        hood.scaleRange(0, 1);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toRadians(0)));
    }

    public void setFlywheelVelocity(double flywheelVelocity) {
        bottomMotor.setVelocity(flywheelVelocity);
        topMotor.setVelocity(flywheelVelocity);
    }

    public void setTurretAngle(double turretAngle) {
        turret1.setPosition(turretAngle);
        turret2.setPosition(turretAngle);
    }

    public void setHoodAngle(double hoodAngle) {
        hood.setPosition(hoodAngle);
    }

    public void setDepollinatorAngle(double depollinatorAngle) {
        depollinator.setPosition(depollinatorAngle);
    }

    public void setBlockerAngle(double blockerAngle) {
        blocker.setPosition(blockerAngle);
    }

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }

    public void setTransferPower(double transferPower) {
        transfer.setPower(transferPower);
    }

    public void resetOdometry() {
        if (gamepad1.startWasPressed()) {
            follower.setPose(new Pose(w, l, Math.toRadians(0)));
        }
    }

    public void varsUpdate() {
        x = follower.pose().x();
        y = follower.pose().y();

        heading = follower.pose().heading();
    }

    public void RCTdriveUpdate() {
        if (!gamepad1.left_trigger_pressed) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            ManualDrive.driveOrHold(follower, powers);
            follower.update();
        }
    }

    public double targetHive() {
        if (gamepad1.xWasPressed()) {targetHive += 1;}
        return targetHive % 2;
    }
    public double getAngleToHive(double x, double y) {
        double angle = 0;
        if (targetHive() == 1) {
            double d_x = 56 - x;
            double d_y = 56 - y;
            angle = Math.toDegrees(Math.atan2(d_y, d_x));

        } else {
            double d_x = 56 - x;
            double d_y = 88 - y;
            angle = Math.toDegrees(Math.atan2(d_y, d_x));
        }
        return angle;
    }
}