package org.firstinspires.ftc.teamcode;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;


public class Robot {
    //Initialize things from hardware map
    DcMotorEx bottomMotor;
    DcMotorEx topMotor;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    Servo turret1;
    Servo turret2;
    Servo blocker;
    Servo depollinator;
    Servo hood;
    

    //Initialize Pedro trackers
    Follower follower;
    Velocity velocity;
    double x;
    double y;
    double w = 7.75;
    double l = 9;
    double heading;
    double targetHeading;
    int intakeTarget;
    double turretRange = 135;


    //Initialize passed values
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Telemetry telemetry;

    //Initialize Shooter Commands
    public Command cmdOpenBlocker;

    public Command cmdCloseBlocker;
    public Command shoot;

    //Initialize Flower Intake Commands
    public Command cmdLowerDepollinator;

    public Command cmdRaiseDepollinator;

    int depollinatorState = 1;
    int hive;

    Timer intakeTimer;
    Timer depollinatorTimer;

    public Robot(HardwareMap hwMap, Gamepad gp1, Telemetry tlm) {
        //Declare passed objects
        hardwareMap = hwMap;
        gamepad1 = gp1;
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
        follower = Constants.create(hardwareMap);
        bottomMotor.setDirection(DcMotorEx.Direction.REVERSE);
        topMotor.setDirection(DcMotorEx.Direction.FORWARD);
        hive = 0;
        
        //Declare Shooter Commands
        cmdOpenBlocker = Command.build()
                .setStart(this::openBlocker)
                .setDone(() -> true)
                .requiring(blocker);

        cmdCloseBlocker = Command.build()
                .setStart(this::closeBlocker)
                .setDone(() -> true)
                .requiring(blocker);

        shoot = sequential(
                cmdOpenBlocker,
                waitMs(1000),
                cmdCloseBlocker
        );

        //Declare Flower Intake Commands
        cmdLowerDepollinator = Command.build()
                .setStart(this::lowerDepollinator)
                .setDone(() -> true)
                .requiring(depollinator);
        cmdRaiseDepollinator = Command.build()
                .setStart(this::raiseDepollinator)
                .setDone(() -> true)
                .requiring(depollinator);

        intakeTimer = new Timer();
        depollinatorTimer = new Timer();
    }
    public void setFlywheelTPS(double TPS) {
        bottomMotor.setVelocity(TPS);
        topMotor.setVelocity(TPS);
    }
    public void sethoodPosition(double angle) {
        hood.setPosition(angle);
    }

    public void raiseDepollinator() {
        depollinator.setPosition(1);
    }

    public void lowerDepollinator() {
        depollinator.setPosition(0);
    }

    public void handleDepollinator(){
        if (gamepad1.aWasPressed()) {
            depollinatorState += 1;
            depollinator.setPosition(depollinatorState % 2);
        }
        if (depollinatorState == 0)
            if (depollinator.getPosition() == 0 && depollinatorTimer.milliseconds()%200 < 100){
                depollinator.setPosition(0.05);
            } else if (depollinator.getPosition() == 0){
                depollinator.setPosition(0);
            }

    }

    public void resetOdometry() {
        if (gamepad1.startWasPressed()) {
            follower.setPose(new Pose(w, l, Math.toRadians(90)));
        }
    }
    public void openBlocker() {
        blocker.setPosition(1);
    }

    public void closeBlocker() {
        blocker.setPosition(0);
    }

    public void blockerUpdate() {
        if (gamepad1.right_trigger_pressed){
            openBlocker();
        }
        else{
            closeBlocker();
        }
    }

    public void varsUpdate() {
        x = follower.pose().x();
        y = follower.pose().y();
        heading = follower.pose().heading();
    }

    public void intake() {
        intake.setVelocity(2250);
    }

    public void outtake() {
        intake.setVelocity(-2250);
    }
    public void handleIntake() {
        if (gamepad1.dpadUpWasPressed()) {
            intake.setPower(1);
        } else if (gamepad1.dpadDownWasPressed()) {
            intake.setPower(-1);
        } else if(gamepad1.yWasPressed()) {
            intake.setPower(0);
        }
    }

    //Field Centric Drive & Turning
    public void FCTdriveUpdate() {
        if (!gamepad1.left_trigger_pressed)
        {
            double hd = heading * 0.5;
            double rx = gamepad1.right_stick_x * Math.cos(hd) + (-gamepad1.right_stick_y * Math.sin(hd));
            double ry = -gamepad1.right_stick_y * Math.cos(hd) - (gamepad1.right_stick_x * Math.sin(hd));

            if (Math.hypot(rx, ry) > 0.12) {
                // Target angle in field coordinates.
                // 0 = +X, PI/2 = +Y, assuming Pedro's standard coordinates.
                targetHeading = Math.atan2(ry, rx);
            }

            // Shortest signed angular difference, in radians.
            double headingError = Math.atan2(
                    Math.sin(targetHeading - hd),
                    Math.cos(targetHeading - hd)
            );
            // Proportional turn control.
            Controller headingCalc = Controller.pid(0.5, 230, 1.5);
            double turn = headingCalc.calculate(targetHeading, headingError);
            // Keep motor command in the valid range.
            turn = Math.max(-1.0, Math.min(1.0, turn));
            if (headingError < 0.01 && headingError > -0.01) {
                turn = 0;
            }

            // Field-centric translation from the left stick.
            DrivePowers powers = ManualDrive.fieldCentric(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    turn,
                    follower.pose().heading()
            );
            follower.manual(powers);
            follower.update();
            telemetry.addData("Turn", turn);
        }
    }
    public void RCTdriveUpdate() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                heading
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }

    public void targetHive() {
        /*
        if (gamepad1.dpadUpWasPressed()) {
            hive = 1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            hive = 0;
        }*/
        if (y < 60){
            hive = 0;
        }
        else if (y > 80){
            hive = 1;
        }
    }
    public double getAngleToHive()
    {
        double hd = Math.toDegrees(heading);
        double dx = (56) - x - (Math.cos(Math.toRadians(hd)) * 3.5);
        double dy = (53 + (37 * hive)) - y - (Math.sin(Math.toRadians(hd)) * 3.5);
        double angle = (Math.toDegrees(Math.atan2(dy,dx)));
        if (hd-angle < 180){
            return (hd - angle);
        }
        else{
            return (-360 + (hd - angle));
        }
    }
    public void aimToHive() {
        if (gamepad1.left_trigger_pressed)
        {
            targetHeading = (0.5 - (getAngleToHive())/(turretRange*2));

            //set the turret angle
            aimTurret(targetHeading);
        }
    }
    public void aimTurret(double power){
        turret1.setPosition(power);
        turret2.setPosition(power);
    }
}
