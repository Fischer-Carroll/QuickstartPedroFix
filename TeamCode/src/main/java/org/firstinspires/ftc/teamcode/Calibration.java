package org.firstinspires.ftc.teamcode;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    double flywheelkP;
    double flywheelkF;
    double targetFlywheelVelocity;
    int[] tuneIncrement;
    int incriment;
    double hoodPosition;
    int blockerState;
    double targetTurretAngle;
    double turretPosition;
    int turretMode;
    String[] turretModeOptions = {"Tune Turret Angle", "Direct Stick Input", "Robot Centric Stick Input", "Field Centric Stick Input"};
    double depollinatorState;
    double intakePower;
    double transferPower;
    double[] flywheelTelemetry;
    double hoodTelemetry;
    double turretTelemetry;
    double stickAngle;
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
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toRadians(0)));

        //Flywheel
        bottomMotor.setDirection(DcMotorEx.Direction.REVERSE);
        topMotor.setDirection(DcMotorEx.Direction.FORWARD);
        tuneIncrement = new int[]{1000, 100, 10, 5};
        incriment = 0;
        targetFlywheelVelocity = 1500;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelkP, 0, 0, flywheelkF);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        getFlywheelTelemetry();

        //Hood
        hoodPosition = 0.5;
        hood.scaleRange(0, 108.8 / 300.0);
        hoodTelemetry = hood.getPosition();
        setHoodPosition(hoodPosition);

        //Blocker
        blockerState = 0;
        blocker.scaleRange(0.2, 0.4);
        setBlockerPosition(blockerState % 2);

        //Turret
        turret1.scaleRange(0, 1);
        turret2.scaleRange(0, 1);
        turretTelemetry = turret1.getPosition();
        turretMode = 0;
        turretPosition = 0.5;
        setTurretPosition(turretPosition);
    }

    // %==========% SHOOTER %==========% //
    public void tuneShooter() { //DO NOT USE WITH tuneFlywheelPIDF() || cycleTurretMode()
        if (gamepad1.startWasPressed()) {
            incriment = (incriment + 1) % tuneIncrement.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            targetFlywheelVelocity += tuneIncrement[incriment];
            setFlywheelVelocity(targetFlywheelVelocity);
        }
        if (gamepad1.dpadUpWasPressed()) {
            targetFlywheelVelocity -= tuneIncrement[incriment];
            setFlywheelVelocity(targetFlywheelVelocity);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            hoodPosition += 0.025;
            setHoodPosition(hoodPosition);
        }
        if (gamepad1.dpadRightWasPressed()) {
            hoodPosition -= 0.025;
            setHoodPosition(hoodPosition);
        }

        telemetry.addLine("Tune Shooter Controls:");
        telemetry.addLine("Tune Incriment: X");
        telemetry.addLine("Tune Flywheel Velocity: dpad up/down");
        telemetry.addLine("Tune Hood Position: dpad left/right");
        telemetry.addLine("Tune Shooter Data:");
        telemetry.addData("Tune Incriment:", tuneIncrement[incriment]);
        telemetry.addData("Measured Flywheel Velocity: ", bottomMotor.getVelocity());
        telemetry.addData("Measured Hood Position", hood.getPosition());
        telemetry.addData("Target Flywheel Velocity: ", targetFlywheelVelocity);
        telemetry.addData("Target Hood Position: ", hoodPosition);
        telemetry.addLine("===============================");

    }

    // %==========% FLYWHEEL %==========% //
    public void setFlywheelVelocity(double targetFlywheelVelocity) {
        setFlywheelVelocity(targetFlywheelVelocity);
    }
    public void tuneFlywheelPIDF() { //DO NOT USE WITH tuneShooter() || cycleTurretMode()
        if (gamepad1.startWasPressed()) {
            incriment = (incriment + 1) % tuneIncrement.length;
        }
        if (gamepad1.leftBumperWasPressed()) {
            flywheelkP += tuneIncrement[incriment];
        }
        if (gamepad1.leftTriggerWasPressed()) {
            flywheelkP -= tuneIncrement[incriment];
        }
        if (gamepad1.rightBumperWasPressed()) {
            flywheelkF += tuneIncrement[incriment];
        }
        if (gamepad1.rightTriggerWasPressed()) {
            flywheelkF -= tuneIncrement[incriment];
        }
        if (gamepad1.dpadUpWasPressed()) {
            targetFlywheelVelocity = 1500;
            setFlywheelVelocity(targetFlywheelVelocity);
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetFlywheelVelocity = 2140;
            setFlywheelVelocity(targetFlywheelVelocity);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelkP, 0, 0, flywheelkF);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Tune Flywheel PIDF Controls:");
        telemetry.addLine("Tune Incriment: X");
        telemetry.addLine("Tune kP: left/right bumper");
        telemetry.addLine("Tune kF: left/right trigger");
        telemetry.addLine("Tune Flywheel Velocity: dpad up/down");
        telemetry.addLine("Tune Flywheel PIDF Data:");
        telemetry.addData("Tune Incriment: ", tuneIncrement[incriment]);
        getFlywheelTelemetry();
        telemetry.addData("Flywheel Velocity Target", flywheelTelemetry[0]);
        telemetry.addData("Flywheel Velocity", flywheelTelemetry[1]);
        telemetry.addData("Flywheel Velocity Error", flywheelTelemetry[2]);
        telemetry.addData("Flywheel kP", flywheelTelemetry[3]);
        telemetry.addData("Flywheel kF", flywheelTelemetry[4]);
        telemetry.addLine("===============================");
    }
    public void getFlywheelTelemetry() {
        flywheelTelemetry = new double[]{
                targetFlywheelVelocity,
                bottomMotor.getVelocity(),
                (bottomMotor.getVelocity() - targetFlywheelVelocity),
                flywheelkP,
                flywheelkF,
                tuneIncrement[incriment]
        };
    }
    /*public void handleDynamicFlywheelVelocity() {
        x = getDistanceToHive();
        hoodPosition = //
        setHoodPosition(hoodPosition);
        targetFlywheelVelocity = //
        setFlywheelVelocity(targetFlywheelVelocity)
    }*/

    // %==========% HOOD %==========% //
    public void setHoodPosition(double hoodPosition) {
        hood.setPosition(hoodPosition);
    }
    public void tuneHoodPosition() {
        if (gamepad1.dpadRightWasPressed()) {
            hoodPosition += 0.025;
            setHoodPosition(hoodPosition);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            hoodPosition -= 0.025;
            setHoodPosition(hoodPosition);
        }

        telemetry.addLine("Hood Tune Controls:");
        telemetry.addLine("Hood Position: dpad right/left");
        telemetry.addData("Hood Position: ", hood.getPosition());
        telemetry.addLine("===============================");
    }

    // %==========% TURRET %==========% //
    public void setTurretPosition(double turretPosition) {
        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
    }
    private void tuneTurretPosition() {
        if (gamepad1.rightStickButtonWasPressed()) {
            turretPosition += 0.025;
            setTurretPosition(turretPosition);
        }
        if (gamepad1.leftStickButtonWasPressed()) {
            turretPosition -= 0.025;
            setTurretPosition(turretPosition);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick button");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addLine("===============================");
    }
    private void manualTurretWithDirectStick() {
        double rx = gamepad1.right_stick_x;

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
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        if (Math.hypot(rx, ry) > 0.12) {
            turretPosition = ((-1 * Math.atan2(ry, rx)) + Math.PI) / (2 * Math.PI);
            setTurretPosition(turretPosition);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick (angle)");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addData("Stick Angle: ", stickAngle);
        telemetry.addLine("===============================");
    }
    private void manualTurretWithFCStickAngle() {
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

        if (Math.hypot(rx, ry) > 0.12) {
            stickAngle = Math.atan2(rx, ry);
            targetTurretAngle = stickAngle - heading;
            turretPosition = targetTurretAngle * ((Math.toRadians(256.5)) / (2 * Math.PI));
            setTurretPosition(turretPosition);
        }

        telemetry.addLine("Turret Tune Controls:");
        telemetry.addLine("Turret Position: right/left stick (field angle)");
        telemetry.addData("Turret Position: ", turret1.getPosition());
        telemetry.addData("Stick Angle: ", stickAngle);
        telemetry.addLine("===============================");
    }
    public void cycleTurretMode() { //DO NOT USE WITH tuneShooter() || tuneFlywheelPIDF()
        if (gamepad1.startWasPressed()) {
            turretMode += 1;
        }
        if (turretMode % 4 == 0) {tuneTurretPosition();}
        if (turretMode % 4 == 1) {manualTurretWithDirectStick();}
        if (turretMode % 4 == 2) {manualTurretWithRCStickAngle();}
        if (turretMode % 4 == 3) {manualTurretWithFCStickAngle();}

        telemetry.addData("Turret Mode: ", turretModeOptions[turretMode % 4]);
        telemetry.addLine("===============================");
    }

    // %==========% BLOCKER %==========% //
    public void setBlockerPosition(double blockerPosition) {
        blocker.setPosition(blockerPosition);
    }
    public void toggleBlocker() {
        if (gamepad1.yWasPressed ()){
            blockerState += 1;
            setBlockerPosition(blockerState % 2);
        }

        telemetry.addLine("Toggle Blocker: Y");
        telemetry.addLine("===============================");
    }

    // %==========% INTAKE %==========% //
    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }
    public void toggleIntake() {
        if (gamepad1.aWasPressed()) {
            intakePower += 1;
            setIntakePower(intakePower % 2);
        }

        telemetry.addLine("Toggle Intake: A");
        telemetry.addLine("===============================");
    }

    // %==========% TRANSFER %==========% //
    public void setTransferPower(double transferPower) {
        transfer.setPower(transferPower);
    }
    public void toggleTransfer() {
        if (gamepad1.bWasPressed()) {
            transferPower += 1;
            setTransferPower(transferPower % 2);
        }

        telemetry.addLine("Toggle Transfer: B");
        telemetry.addLine("===============================");
    }

    // %==========% DEPOLLINATOR %==========% //
    public void setDepollinatorPosition(double depollinatorPosition) {
        depollinator.setPosition(depollinatorPosition);
    }
    public void toggleDepollinator() {
        if (gamepad1.xWasPressed()) {
            depollinatorState += 1;
            setDepollinatorPosition(depollinatorState % 2);
        }
        telemetry.addLine("Toggle Depollinator: X");
        telemetry.addLine("===============================");
    }

    // %==========% ODOMETRY %==========% //
    public void resetOdometry() {
        if (gamepad1.backWasPressed()) {
            follower.setPose(new Pose(w, l, Math.toRadians(0)));
        }

        telemetry.addLine("Reset Odometry: Back");
        telemetry.addLine("===============================");
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
}