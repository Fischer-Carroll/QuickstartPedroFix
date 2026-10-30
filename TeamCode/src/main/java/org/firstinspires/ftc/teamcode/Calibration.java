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
    double targetHive;
    double distance;
    double flywheelkP;
    double flywheelkF;
    double targetFlywheelVelocity;
    int[] tuneIncrement;
    int incriment;
    double hoodAngle;
    int blockerState;
    double turretAngle;
    double depollinatorState;
    double intakePower;
    double transferPower;
    double[] flywheelTelemetry;
    double hoodTelemetry;
    double turretTelemetry;
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
        hoodAngle = 0;
        hood.scaleRange(0, 108.8 / 300.0);
        hoodTelemetry = hood.getPosition();
        //Blocker
        blockerState = 0;
        blocker.scaleRange(0.2, 0.4);
        //Turret
        turret1.scaleRange(0, 1);
        turret2.scaleRange(0, 1);
        turretTelemetry = turret1.getPosition();
    }

    //===========Shooter==========//
    public void setFlywheelVelocity(double flywheelVelocity) {
        bottomMotor.setVelocity(flywheelVelocity);
        topMotor.setPower(bottomMotor.getPower());
    }

    public void tuneFlywheelVelocity() {
        if (gamepad1.xWasPressed()) {
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

        getFlywheelTelemetry();
        telemetry.addData("Flywheel Velocity Target", flywheelTelemetry[0]);
        telemetry.addData("Flywheel Velocity", flywheelTelemetry[1]);
        telemetry.addData("Flywheel Velocity Error", flywheelTelemetry[2]);
        telemetry.addData("Flywheel kP", flywheelTelemetry[3]);
        telemetry.addData("Flywheel kF", flywheelTelemetry[4]);
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

    public void tuneHoodAngle() {
        if (gamepad1.dpadLeftWasPressed()) {
            hoodAngle += 0.025;
            setHoodAngle(hoodAngle);
        }
        if (gamepad1.dpadRightWasPressed()) {
            hoodAngle -= 0.025;
            setHoodAngle(hoodAngle);
        }
        telemetry.addData("Hood Angle: ", hood.getPosition());
    }

    public void tuneTurretAngle() {
        if (gamepad1.startWasPressed()) {
            turretAngle += 0.025;
            setTurretAngle(turretAngle);
        }
        if (gamepad1.backWasPressed()) {
            turretAngle -= 0.025;
            setTurretAngle(turretAngle);
        }
        telemetry.addData("Turret Angle: ", turret1.getPosition());
    }

    public void manualTurretWithDirectStick() {
        double rx = gamepad1.right_stick_x;
        if (Math.abs(rx) > 0.12) {
            turretAngle += 0.01 * Math.signum(rx);
            setTurretAngle(turretAngle);
        }
    }
    public void manualTurretWithStickAngle() {
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;
        if (Math.hypot(rx, ry) > 0.12) {
            turretAngle = Math.atan2(ry, rx);
            setTurretAngle(turretAngle);
        }
    }

    public void toggleBlocker() {
        if (gamepad1.yWasPressed ()){
            blockerState += 1;
            setBlockerAngle(blockerState % 2);
        }
    }

    public void toggleIntake() {
        if (gamepad1.aWasPressed()) {
            intakePower += 1;
            setIntakePower(intakePower % 2);
        }
    }

    public void toggleTransfer() {
        if (gamepad1.bWasPressed()) {
            transferPower += 1;
            setTransferPower(transferPower % 2);

        }
    }

    public void toggleDepollinator() {
        if (gamepad1.xWasPressed()) {
            depollinatorState += 1;
            setDepollinatorAngle(depollinatorState % 2);

        }
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
}