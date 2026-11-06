package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import android.os.CpuUsageInfo;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

public class Calibration {
    // INITIALISING OBJECTS //
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    DcMotorEx bottomMotor;
    DcMotorEx topMotor;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    Servo turret1;
    Servo turret2;
    Servo blocker;
    Servo depollinator;
    Servo hood;
    VoltageSensor voltageSensor;
    Timer shootTimer = new Timer();
    com.pedropathing.utils.Timer shootWaitTimer = new Timer();

    public Calibration(HardwareMap hwMap, Gamepad gp1, Gamepad gp2, Telemetry tlm) {
        // PASSED OBJECTS //
        hardwareMap = hwMap;
        gamepad1 = gp1;
        gamepad2 = gp2;
        telemetry = tlm;

        // HARDWARE MAP //
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depollinator = hardwareMap.get(Servo.class, "depollinator");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toRadians(0)));

        // SHOOTER //
        Timer shootTimer = new Timer();
        Timer shootWaitTimer = new Timer();
        shootTimer.reset();
        shootWaitTimer.reset();
        setTransferPower(idleTransferPower);

        // FLYWHEEL //
        bottomMotor.setDirection(DcMotorEx.Direction.REVERSE);
        topMotor.setDirection(DcMotorEx.Direction.FORWARD);
        bottomMotor.setMode(RUN_USING_ENCODER);
        topMotor.setMode(RUN_USING_ENCODER);
        targetFlywheelVelocity = 0;
        targetFlywheelPower = 0;

        // HOOD //
        hood.scaleRange(0.2,0.625);
        setHoodPosition(0.5);

        // BLOCKER //
        blocker.scaleRange(0.2, 0.4);
        setBlockerPosition(0);

        // TURRET //
        turret1.scaleRange(0, 1);
        turret2.scaleRange(0, 1);
        turretMode = 0;
        setTurretPosition(0.5);

        // DEPOLLINATOR //
        depollinator.scaleRange(0, 1);
        setDepollinatorPosition(1);

        // MISCELLANEOUS //
        tuneIncrement = new int[]{1000, 100, 10, 5};
        increment = 0;
    }

    // %==========% MISCELLANEOUS %==========% //
    int[] tuneIncrement;
    int increment;
    double stickAngle;
    double rx;
    double ry;

    public void getStickAngle() {
        rx = gamepad1.right_stick_x;
        ry = -gamepad1.right_stick_y;
        stickAngle = Math.atan2(rx, ry);

        telemetry.addData("Stick Angle", Math.toDegrees(stickAngle));
        telemetry.addLine("===============================");
    }

    // %==========% SHOOTER %==========% //
    double shootDelay = 500; //ms
    double shootingTransferPower = 0.7;
    double idleTransferPower = 0.3;
    int transferTuningMode = 0;
    private void tuneIdleTransferPower() {
        if (gamepad1.rightBumperWasPressed()) {
            idleTransferPower += 0.05;
        }
        if (gamepad1.leftBumperWasPressed()) {
            idleTransferPower -= 0.05;
        }
    }
    private void tuneShootingTransferPower() {
        if (gamepad1.rightBumperWasPressed()) {
            shootingTransferPower += 0.05;
        }
        if (gamepad1.leftBumperWasPressed()) {
            shootingTransferPower -= 0.05;
        }
    }
    public void cycleTransferPowerTuningModes() {
        telemetry.addLine("Transfer Power Tuning");
        if (gamepad1.startWasPressed()) {
            transferTuningMode += 1;
        }
        if (transferTuningMode % 2 == 0) {
            tuneIdleTransferPower();
            telemetry.addLine("Tuning Idle Power");
            telemetry.addData("Idle Transfer Power", idleTransferPower);
        }
        if (transferTuningMode % 2 == 1) {
            tuneShootingTransferPower();
            telemetry.addLine("Tuning Shooting Power");
            telemetry.addData("Shooting Transfer Power", shootingTransferPower);
        }
        telemetry.addLine("===============================");
    }
    public void testShootControls() {
        if (gamepad1.right_trigger_pressed) {
            if (shootTimer.milliseconds() > shootDelay) {
                setTransferPower(shootingTransferPower);
                shootWaitTimer.reset();
                shootTimer.reset();
            }
        } else {
            if (transferPower > idleTransferPower) {
                setTransferPower(idleTransferPower);
            }
        }
    }
    public void tuneShooter() {
        if (gamepad1.startWasPressed()) {
            increment = (Math.abs(increment + 1)) % tuneIncrement.length;
        }
        if (gamepad1.backWasPressed()) {
            increment = (Math.abs(increment - 1)) % tuneIncrement.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            setFlywheelVelocity(targetFlywheelVelocity += tuneIncrement[increment]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            setFlywheelVelocity(targetFlywheelVelocity -= tuneIncrement[increment]);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            setHoodPosition(hoodPosition += 0.25);
        }
        if (gamepad1.dpadRightWasPressed()) {
            setHoodPosition(hoodPosition -= 0.25);
        }

        telemetry.addLine("Tune Shooter Controls:");
        telemetry.addLine("Tune increment: X");
        telemetry.addLine("Tune Flywheel Velocity: dpad up/down");
        telemetry.addLine("Tune Hood Position: dpad left/right");
        telemetry.addLine("Tune Shooter Data:");
        telemetry.addData("Tune increment:", tuneIncrement[increment]);
        telemetry.addData("Corrected Flywheel Velocity", correctedFlywheelVelocity);
        telemetry.addData("Measured Flywheel Velocity: ", bottomMotor.getVelocity());
        telemetry.addData("Measured Hood Position", hood.getPosition());
        telemetry.addData("Target Flywheel Velocity: ", targetFlywheelVelocity);
        telemetry.addData("Target Hood Position: ", hoodPosition);
        telemetry.addLine("===============================");
    }

    // %==========% FLYWHEEL %==========% //
    static double targetFlywheelVelocity;
    static double targetFlywheelPower;

    private void setFlywheelVelocity(double targetFlywheelVelocity) {
        Calibration.targetFlywheelVelocity = targetFlywheelVelocity;
        bottomMotor.setVelocity(targetFlywheelVelocity);
        topMotor.setVelocity(targetFlywheelVelocity);
    }

    private void setFlywheelPower(double targetFlywheelPower) {
        Calibration.targetFlywheelPower = targetFlywheelPower;
        bottomMotor.setPower(targetFlywheelPower);
        topMotor.setPower(targetFlywheelPower);
    }

    //PID Control

    double highVelocity = 2200.0;
    double lowVelocity = 1500.0;
    double correctedFlywheelVelocity;
    double flywheelVelocityError;
    double P = 0;
    double I = 0;
    double D = 0;
    double F = 12;

    public void pidCorrection() {
        flywheelVelocityError = topMotor.getVelocity() - targetFlywheelVelocity;
        correctedFlywheelVelocity = targetFlywheelVelocity - (flywheelVelocityError * 2);
    }

    private void updatePIDFCoefficients() {
        PIDFCoefficients pidf = new PIDFCoefficients(P,I,D,F);
        topMotor.setPIDFCoefficients(RUN_USING_ENCODER,pidf);
        bottomMotor.setPIDFCoefficients(RUN_USING_ENCODER,pidf);
    }
    
    public void handlePIDFFlywheel() {
        double topMotorVelocity = topMotor.getVelocity();
        double bottomMotorVelocity = bottomMotor.getVelocity();
        double avgFlywheelVelocity = (topMotorVelocity + bottomMotorVelocity) / 2;
        double error = targetFlywheelVelocity - avgFlywheelVelocity;
        double errorPercent = (error / targetFlywheelVelocity) * 100;
        F = voltageSensor.getVoltage();

        if (gamepad1.startWasPressed()) {
            increment = (increment + 1) % tuneIncrement.length;
        }
        if (gamepad1.leftBumperWasPressed()) {
            P += (tuneIncrement[increment] / 1000.0);
        }
        if (gamepad1.leftTriggerWasPressed()) {
            P -= (tuneIncrement[increment] / 1000.0);
        }
        if (gamepad1.dpadDownWasPressed()) {
            setFlywheelVelocity(lowVelocity);
        }
        if (gamepad1.dpadUpWasPressed()) {
            setFlywheelVelocity(highVelocity);
        }

        updatePIDFCoefficients();

        telemetry.addLine("Tune Flywheel Controls:");
        telemetry.addLine("Tune Increment: X");
        telemetry.addLine("Tune kP: left/right bumper");
        telemetry.addLine("Tune kF: left/right trigger");
        telemetry.addLine("Tune Flywheel Velocity: dpad up/down");
        telemetry.addLine("Tune Flywheel Data:");
        telemetry.addData("Tune Increment: ", (tuneIncrement[increment] / 1000.0));
        telemetry.addData("Flywheel Velocity Target", targetFlywheelVelocity);
        telemetry.addData("Flywheel Velocity", avgFlywheelVelocity);
        telemetry.addData("Top Motor Velocity", topMotorVelocity);
        telemetry.addData("Bottom Motor Velocity", bottomMotorVelocity);
        telemetry.addData("Flywheel Velocity Error", error);
        telemetry.addData("Flywheel Velocity Error", errorPercent);
        telemetry.addData("Flywheel Power", (topMotor.getPower()));
        telemetry.addData("Flywheel P", P);
        telemetry.addData("Flywheel F", F);
        telemetry.addLine("===============================");
    }

    public void passivePIDFFlywheel() {
        double P = 0.9;
        double I = 0.0;
        double D = 0.0;
        double F = (voltageSensor.getVoltage() / 12) * 12;

        PIDFCoefficients pidf = new PIDFCoefficients(P,I,D,F);
        topMotor.setPIDFCoefficients(RUN_USING_ENCODER,pidf);
        bottomMotor.setPIDFCoefficients(RUN_USING_ENCODER,pidf);
    }
    
    //for future use once polynomial regression is used to find correlation between distance and shooter vars
    /*public void handleDynamicFlywheelVelocity() {
        x = getDistanceToHive();
        hoodPosition = //calc for hood position
        setHoodPosition(hoodPosition);
        targetFlywheelVelocity = //calc for flywheel velocity
        setFlywheelVelocity(targetFlywheelVelocity)
    }*/

    // %==========% HOOD %==========% //
    static double hoodPosition;

    private void setHoodPosition(double hoodPosition) {
        Calibration.hoodPosition = hoodPosition;
        hood.setPosition(hoodPosition);
    }

    public void tuneHoodPosition() {
        if (gamepad1.dpadRightWasPressed()) {
            setHoodPosition(hoodPosition -= 0.025);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            setHoodPosition(hoodPosition += 0.025);
        }

        telemetry.addLine("Hood Tune Controls:");
        telemetry.addLine("Hood Position: dpad right/left");
        telemetry.addData("Hood Position: ", hood.getPosition());
        telemetry.addLine("===============================");
    }

    // %==========% TURRET %==========% //
    double targetTurretAngle;
    static double turretPosition;
    int turretMode;
    String[] turretModeOptions = {"Tune Turret Angle", "Direct Stick Input", "Robot Centric Stick Input", "Field Centric Stick Input"};

    private void setTurretPosition(double turretPosition) {
        Calibration.turretPosition = turretPosition;
        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
    }

    public void tuneTurretPosition() {
        if (gamepad1.rightStickButtonWasPressed()) {
            setTurretPosition(turretPosition += 0.025);
        }
        if (gamepad1.leftStickButtonWasPressed()) {
            setTurretPosition(turretPosition -= 0.025);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick button");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addLine("===============================");
    }

    private void manualTurretWithDirectStick() {
        if (Math.abs(rx) > 0.12) {
            turretPosition += 0.01 * Math.signum(rx);
            setTurretPosition(turretPosition);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick (direction)");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addData("Stick Magnitude: ", rx);
        telemetry.addLine("===============================");
    }

    private void manualTurretWithRCStickAngle() {
        if (Math.hypot(rx, ry) > 0.12) {
            setTurretPosition(((-1.0 * stickAngle) + Math.PI) / (2.0 * Math.PI));
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick (angle)");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addData("Stick Angle: ", stickAngle);
        telemetry.addLine("===============================");
    }

    private void manualTurretWithFCStickAngle() {
        if (Math.hypot(rx, ry) > 0.12) {
            targetTurretAngle = stickAngle - heading;
            setTurretPosition(targetTurretAngle / (Math.toRadians(256.5) / Math.toRadians(360.0)));
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick (field angle)");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addData("Stick Angle: ", stickAngle);
        telemetry.addLine("===============================");
    }

    public void cycleTurretMode() {
        if (gamepad1.startWasPressed()) {
            turretMode += 1;
        }
        if (turretMode % 4 == 0) {
            tuneTurretPosition();
        }
        if (turretMode % 4 == 1) {
            manualTurretWithDirectStick();
        }
        if (turretMode % 4 == 2) {
            manualTurretWithRCStickAngle();
        }
        if (turretMode % 4 == 3) {
            manualTurretWithFCStickAngle();
        }

        telemetry.addData("Turret Mode: ", turretModeOptions[turretMode % 4]);
        telemetry.addLine("===============================");
    }

    // %==========% BLOCKER %==========% //
    static double blockerPosition;
    private void setBlockerPosition(double blockerPosition) {
        Calibration.blockerPosition = (int) blockerPosition;
        blocker.setPosition(blockerPosition);
    }

    public void tuneBlockerPosition() {
        if (gamepad1.dpadLeftWasPressed()) {
            setBlockerPosition(blockerPosition += 0.025);
        }
        if (gamepad1.dpadRightWasPressed()) {
            setBlockerPosition(blockerPosition -= 0.025);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick button");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addLine("===============================");
    }

    public void toggleBlocker() {
        if (gamepad1.yWasPressed()) {
            setBlockerPosition((blockerPosition += 1) % 2);
        }

        telemetry.addLine("Toggle Blocker: Y");
        telemetry.addLine("===============================");
    }

    // %==========% INTAKE %==========% //
    static double intakePower;
    private void setIntakePower(double intakePower) {
        Calibration.intakePower = intakePower;
        intake.setPower(intakePower);
    }

    public void toggleIntake() {
        if (gamepad1.aWasPressed()) {
            setIntakePower((intakePower += 1) % 2);
        }

        telemetry.addLine("Toggle Intake: A");
        telemetry.addLine("===============================");
    }

    // %==========% TRANSFER %==========% //
    static double transferPower;
    private void setTransferPower(double transferPower) {
        Calibration.transferPower = transferPower;
        transfer.setPower(transferPower);
    }

    public void toggleTransfer() {
        if (gamepad1.bWasPressed()) {
            setTransferPower((transferPower += 1) % 2);
        }

        telemetry.addLine("Toggle Transfer: B");
        telemetry.addLine("===============================");
    }

    // %==========% DEPOLLINATOR %==========% //
    int depollinatorState;
    static double depollinatorPostion;
    private void setDepollinatorPosition(double depollinatorPosition) {
        Calibration.depollinatorPostion = depollinatorPosition;
        depollinator.setPosition(depollinatorPosition);
    }

    public void toggleDepollinator() {
        if (gamepad1.xWasPressed()) {
            setDepollinatorPosition((depollinatorState += 1) % 2);
        }

        telemetry.addLine("Toggle Depollinator: X");
        telemetry.addLine("===============================");
    }

    public void tuneDepollinatorPosition() {
        if (gamepad1.dpadRightWasPressed()) {
            setDepollinatorPosition(depollinatorPostion -= 0.025);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            setDepollinatorPosition(depollinatorPostion += 0.025);
        }

        telemetry.addLine("Depollinator Tune Controls:");
        telemetry.addLine("Depollinator Position: dpad right/left");
        telemetry.addData("Depollinator Position: ", depollinator.getPosition());
        telemetry.addLine("===============================");
    }

    // %==========% DRIVE %==========% //
    Follower follower;
    double x;
    double y;
    double w = 7.75;
    double l = 9.0;
    double heading;
    DrivePowers powers;
    public Pose resetPose = new Pose(w, l, 90);

    public void resetOdometry() {
        if (gamepad1.backWasPressed()) {
            follower.setPose(resetPose);
            follower.hold(resetPose);
        }
    }

    public void varsUpdate() {
        x = follower.pose().x();
        y = follower.pose().y();
        heading = follower.pose().heading();

        telemetry.addData("Robot Heading: ", heading);
        telemetry.addLine("===============================");
    }

    public void RCTdriveUpdate() {
        if (!gamepad1.left_trigger_pressed) {
            if (gamepad1.backWasPressed()) {
                resetOdometry();
            } else {
                powers = ManualDrive.fieldCentric(
                        gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        follower.pose().heading()
                );
            }

            ManualDrive.driveOrHold(follower, powers);
            follower.update();
        }

        telemetry.addLine("Reset Odometry: Back");
        telemetry.addData("Heading: ", follower.pose().heading());
        telemetry.addLine("===============================");
    }
}