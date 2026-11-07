package org.firstinspires.ftc.teamcode;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public Pose resetPose = new Pose(24,24,Math.toRadians(90));


    //Initialize passed values
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Telemetry telemetry;

    //Initialize Shooter Commands
    public Command cmdOpenBlocker;
    public Command cmdCloseBlocker;
    public Command cmdShoot;
    public Command cmdPrimeTurret;
    
    //Initialize Flower Intake Commands
    public Command cmdLowerDepollinator;
    public Command cmdRaiseDepollinator;

    //Initialize Intake commands
    public Command cmdIntake;
    public Command cmdStopIntake;
    

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

        //Assign directions and behaviors to motors
        follower = Constants.create(hardwareMap);
        bottomMotor.setDirection(DcMotorEx.Direction.REVERSE);
        topMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        cmdPrimeTurret = Command.build()
                .setStart(this::primeTurret)
                .setDone(() -> true)
                .requiring(turret1,turret2,transfer,bottomMotor,topMotor);

        cmdShoot = sequential(
                cmdPrimeTurret,
                waitMs(500),
                cmdOpenBlocker,
                waitMs(750),
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

        cmdIntake = Command.build()
                .setStart(this::intake)
                .setDone(() -> true)
                .requiring(intake);
        
        cmdStopIntake = Command.build()
                .setStart(this::notake)
                .setDone(() -> true)
                .requiring(intake);
        
        intakeTimer = new Timer();
        depollinatorTimer = new Timer();
    }
    public void initialiseServoScales() {
        blocker.scaleRange(0.21, 0.4);
        depollinator.scaleRange(0.227, 0.57);
        hood.scaleRange(0.2,0.625);
        turret1.scaleRange(0.025,0.975);
        turret2.scaleRange(0.025,0.975);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initialiseActuatorPositions() {
        setFlywheelTPS(2140);
        sethoodPosition(0.5);
        aimTurret(0.5);
        transfer.setPower(1);
        intake.setPower(1);
        depollinator.setPosition(depollinatorState);
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
    public void depollinatorControl(){
        if (gamepad1.yWasPressed()) {
            depollinatorState += 1;
            depollinator.setPosition(depollinatorState % 2);
        }
        if (depollinatorState%2 == 0)
            if (depollinatorTimer.milliseconds()%200 < 100){
                depollinator.setPosition(0.04);
            } else {
                depollinator.setPosition(0);
            }

    }
    public void odoResetControl() {
        if (gamepad1.startWasPressed()) {
            follower.setPose(resetPose);
            follower.hold(resetPose);
        }
    }
    public void openBlocker() {
        blocker.setPosition(1);
    }
    public void closeBlocker() {
        blocker.setPosition(0);
    }
    public void blockerControl() {
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
    public void loopUpdate(){
        varsUpdate();
        targetHive();
        telemetry.update();
    }
    public void intake() {
        intake.setPower(1);
    }
    public void outtake() {
        intake.setPower(-1);
    }
    public void notake() {
        intake.setPower(0);
    }
    public void intakeControl() {
        if (gamepad1.leftBumperWasPressed()){
            if (intakeTarget != -1){
                intakeTimer.reset();
            }
            intakeTarget = -1;
        }
        if (gamepad1.rightBumperWasPressed()){
            if (intakeTarget != 1){
                intakeTimer.reset();
            }
            intakeTarget = 1;
        }

        if (intakeTimer.seconds() < 1){
            double targetPower = (intakeTimer.seconds() - 0.5)*intakeTarget*2;
            intake.setPower(targetPower);
        }
    }
    //Field Centric Drive & Turning
    public void FCTdriveUpdate() {
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
    public void driveControl() {
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
    public double getAngleToHive() {
        double hd = Math.toDegrees(heading);
        double dx = (58) - x - (Math.cos(Math.toRadians(hd)) * 3.5);
        double dy = (53 + (37 * hive)) - y - (Math.sin(Math.toRadians(hd)) * 3.5) + Math.abs(dx/25);
        double angle = (Math.toDegrees(Math.atan2(dy,dx)));
        if (hd-angle < 180){
            return (hd - angle);
        }
        else{
            return (-360 + (hd - angle));
        }
    }
    public void autoAimTurret(){
        targetHeading = (0.5 - (getAngleToHive())/(turretRange*2));
        
        //set the turret angle
        aimTurret(targetHeading);
    }
    public void aimTurret(double power){
        turret1.setPosition(power);
        turret2.setPosition(power);
    }
    public int calcFlywheelTPS() {
        double x = calcHiveDistance();
        return (int)((1052*x*x*x*x - 4823*x*x*x + 7978*x*x - 5447*x + 3286) * 1.01);
    }
    public double calcHoodAngle(){
        double x = calcHiveDistance();
        return (-0.9156*x*x*x*x + 4.322*x*x*x - 7.229*x*x + 4.634*x + 0.01);
    }
    public double calcHiveDistance(){
        double hd = Math.toDegrees(heading);
        double dx = (58) - x - (Math.cos(Math.toRadians(hd)) * 3.5);
        double dy = (53 + (37 * hive)) - y - (Math.sin(Math.toRadians(hd)) * 3.5) + Math.abs(dx/25);
        return (Math.hypot(dy,dx))/39.37;

    }
    public void primeTurret(){
        autoAimTurret();
        setFlywheelTPS(calcFlywheelTPS());
        sethoodPosition(calcHoodAngle());
    }
    private void idleTurret(){
        aimTurret(0.5);
        setFlywheelTPS(2000);
        sethoodPosition(0.5);
    }
    //stuff imported from calibration
    double shootDelay = 500; //ms
    public static double transferPower;
    double shootingTransferPower = 0.725;
    double idleTransferPower = 0.3;
    boolean primed = false;

    Timer shootTimer = new Timer();
    private void setTransferPower(double transferPower) {
        Robot.transferPower = transferPower;
        transfer.setPower(transferPower);
    }

    public void shootingControl() {
        if (gamepad1.leftTriggerWasPressed()) {
            setTransferPower(shootingTransferPower);
        }
        else if (gamepad1.leftTriggerWasReleased()){
            setTransferPower(idleTransferPower);
            idleTurret();
        }
        if (gamepad1.left_trigger_pressed && !follower.holding()){
            primeTurret();
            primed = false;
        }
        if (follower.holding() && !primed){
            primeTurret();
            primed = true;
        }
    }
}
