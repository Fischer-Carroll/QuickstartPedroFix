package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Test {
    // Miscellaneous
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    Follower follower;

    // Hardware
    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    DcMotorEx taker;
    DcMotorEx transfer;
    Servo turret1;
    Servo turret2;
    Servo hood;
    Servo blocker;
    Servo depollinator;

    // Timers
    Timer runtime = new Timer();
    Timer depollinatorTimer = new Timer();
    Timer takerTimer = new Timer();

    public Test(HardwareMap hwMap, Gamepad gp1, Gamepad gp2, Telemetry tlm) {
        // Passed Objects
        hardwareMap = hwMap;
        gamepad1 = gp1;
        gamepad2 = gp2;
        telemetry = tlm;

        // Hardware Map
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        taker = hardwareMap.get(DcMotorEx.class, "taker");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depollinator = hardwareMap.get(Servo.class, "depollinator");
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(24,24,Math.toRadians(90)));

        // Configurations
        follower = Constants.create(hardwareMap);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        taker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Scales
        turret1.scaleRange(0.025,0.975);
        turret2.scaleRange(0.025,0.975);
        hood.scaleRange(0.2,0.625);
        blocker.scaleRange(0.21,0.4);
        depollinator.scaleRange(0.227,0.57);
    }

    public void start() {
        runtime.reset();

        // Depollinator
        depollinatorTimer.reset();
        depollinatorUp();

        // Taker
        takerTimer.reset();
        targetTakerPower = 0;
        intake();

        // Shooter
        shooterIdle();
    }

    // Commands
    public Command cmdBlockerUp;
    public Command cmdBlockerDown;

    public Command cmdActivateShooter;
    public Command cmdShoot;

    public Command cmdDepollinatorUp;
    public Command cmdDepollinatorDown;

    public Command cmdIntake;
    public Command cmdNulltake;

    public void Commands() {
        // Blocker
        cmdBlockerUp = Command.build()
                .setStart(this::blockerUp)
                .setDone(() -> true)
                .requiring(blocker);
        cmdBlockerDown = Command.build()
                .setStart(this::blockerDown)
                .setDone(() -> true)
                .requiring(blocker);

        // Shooter
        cmdActivateShooter = Command.build()
                .setStart(this::commandShooterActive)
                .setDone(() -> true)
                .requiring(turret1,turret2,transfer,flywheel1,flywheel2,hood);
        cmdShoot = sequential(
                cmdActivateShooter,
                waitMs(500),
                cmdBlockerUp,
                waitMs(750),
                cmdBlockerDown
        );

        // Depollinator
        cmdDepollinatorDown = Command.build()
                .setStart(this::depollinatorDown)
                .setDone(() -> true)
                .requiring(depollinator);
        cmdDepollinatorUp = Command.build()
                .setStart(this::depollinatorUp)
                .setDone(() -> true)
                .requiring(depollinator);

        // Taker
        cmdIntake = Command.build()
                .setStart(this::intake)
                .setDone(() -> true)
                .requiring(taker);
        cmdNulltake = Command.build()
                .setStart(this::nulltake)
                .setDone(() -> true)
                .requiring(taker);
    }

    // Flywheel
    double flywheelVelocity;
    final double idleFlywheelVelocity = 2080;

    private void setFlywheelVelocity(double velocity) {
        flywheelVelocity = velocity;
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }
    private void setDynamicFlywheelVelocity() {
        double x = getHiveDistance();
        setFlywheelVelocity((int)((1052*x*x*x*x - 4823*x*x*x + 7978*x*x - 5447*x + 3286) * 1.01));
    }

    // Hood
    double hoodPosition;
    final double idleHoodPosition = 0.75;

    private void setHoodPosition(double position) {
        hoodPosition = position;
        hood.setPosition(position);
    }
    private void setDynamicHoodPosition(){
        double x = getHiveDistance();
        setHoodPosition(-0.9156*x*x*x*x + 4.322*x*x*x - 7.229*x*x + 4.634*x + 0.01);
    }

    // Turret
    double turretPosition;
    final double idleTurretPosition = 0.5;
    final double turretFieldOfFire = 135; //deg in each direction

    private void setTurretPosition(double position) {
        turretPosition = position;
        turret1.setPosition(position);
        turret2.setPosition(position);
    }
    private void aimTurret() {
        setTurretPosition(0.5 - (getHiveAngle() / (turretFieldOfFire * 2)));
    }
    private double getTurretPosition() { //returns turret position in degrees
        return (turretPosition - 0.5) * 2 * turretFieldOfFire;
    }

    // Taker (formerly referred to as the intake)
    double takerPower;
    double targetTakerPower;
    final double intakePower = 1;
    final double outtakePower = -1;

    private void setTakerPower(double power) {
        takerPower = power;
        taker.setPower(power);
    }
    private void intake() {setTakerPower(intakePower);}
    private void outtake() {setTakerPower(outtakePower);}
    private void nulltake() {setTakerPower(0);}
    public void takerControl() {
        if (gamepad1.rightBumperWasPressed()) {
            if (targetTakerPower != intakePower) {
                takerTimer.reset();
            } 
            targetTakerPower = intakePower;
        } if (gamepad1.leftBumperWasPressed()) {
            if (targetTakerPower != outtakePower) {
                takerTimer.reset();
            } 
            targetTakerPower = outtakePower;
        } if (gamepad1.xWasPressed()) {
            targetTakerPower = 0;
            setTakerPower(targetTakerPower);
        } if (takerPower == 0 && targetTakerPower != 0) {
            setTakerPower(targetTakerPower);
        } else if (takerTimer.seconds() < 1) {
            setTakerPower((takerTimer.seconds() - 0.5) * targetTakerPower*2);
        }
    }
    public void takerControlV2() {
        if (gamepad1.rightBumperWasPressed()) {
            targetTakerPower = intakePower;
            takerTimer.reset();
        } if (gamepad1.leftBumperWasPressed()) {
            targetTakerPower = outtakePower;
            takerTimer.reset();
        } if (gamepad1.xWasPressed()) {
            setTakerPower(0);
        } if (takerTimer.seconds() < 1 && runtime.seconds() > 1) {
            setTakerPower(-targetTakerPower + (Math.signum(targetTakerPower) * takerTimer.seconds() * -2));
        }
    }
    public void takerTelemetry() {
        telemetry.addLine("Intake Controls: ");
        telemetry.addLine("Intake - Right Bumper");
        telemetry.addLine("Outtake - Left Bumper");
        telemetry.addLine("Nulltake - X");
        telemetry.addData("Taker Power", taker.getPower());
        telemetry.addLine("==========================================");
    }

    // Transfer
    double transferPower;
    final double idleTransferPower = 0.3;
    final double activeTransferPower = 0.725;

    private void setTransferPower(double power) {
        transferPower = power;
        transfer.setPower(power);
    }

    // Blocker
    double blockerPosition;
    int blockerUpPosition = 1;
    int blockerDownPosition = 0;
    private void setBlockerPosition(double position) {
        blockerPosition = position;
        blocker.setPosition(position);
    }
    private void blockerUp() {setBlockerPosition(blockerUpPosition);}
    private void blockerDown() {setBlockerPosition(blockerDownPosition);}
    public void blockerControl() {
        if (gamepad1.right_trigger_pressed) {blockerUp();}
        else {blockerDown();}
    }

    // Depollinator
    double depollinatorPosition;
    int depollinatorState = 0;
    final double depollinatorUpPosition = 1;
    final double depollinatorDownPosition = 0.04;
    final double[] depollinatorStates = new double[] {depollinatorUpPosition, depollinatorDownPosition};
    final double depollinatorJiggleValue = 0.03;

    private void setDepollinatorPosition(double position) {
        depollinatorPosition = position;
        depollinator.setPosition(position);
    }
    private void depollinatorUp() {setDepollinatorPosition(depollinatorUpPosition);}
    private void depollinatorDown() {setDepollinatorPosition(depollinatorDownPosition);}
    private void depollinatorJiggle() {
        if (depollinatorPosition == depollinatorDownPosition) {
            if (depollinatorTimer.milliseconds() % 200 < 100) {
                setDepollinatorPosition(depollinatorDownPosition + depollinatorJiggleValue);
            } else {
                setDepollinatorPosition(depollinatorDownPosition - depollinatorJiggleValue);
            }
        }
    }
    public void depollinatorControl() {
        if (gamepad1.yWasPressed()) {
            depollinatorState += 1;
            setDepollinatorPosition(depollinatorStates[depollinatorState]);
        }
        depollinatorJiggle();
    }
    public void depollinatorTelemetry() {
        telemetry.addLine("Depollinator Controls: ");
        telemetry.addLine("Toggle Depollinator: Y");
        telemetry.addLine("==========================================");
    }

    // Drive
    double x;
    double y;
    double heading;
    public Pose odometryResetPose = new Pose(24,24,Math.toRadians(90));

    private void positionUpdate() {
        x = follower.pose().x();
        y = follower.pose().y();
        heading = follower.pose().heading();
    }
    public void odometryResetControl() {
        if (gamepad1.startWasPressed()) {
            follower.setPose(odometryResetPose);
            follower.hold(odometryResetPose);
        }
    }
    public void driveControl() {
        positionUpdate();
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                heading
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }
    public void driveTelemetry() {
        telemetry.addLine("Drive Controls: ");
        telemetry.addLine("Field Centric Driving - Left Stick");
        telemetry.addLine("Robot Centric Turning - Right Stick");
        telemetry.addLine("Reset Odometry - Start");
        telemetry.addLine("(Odometry Reset Position is at (24,24) facing 90 degrees)");
        telemetry.addData("X ", x);
        telemetry.addData("Y ", y);
        telemetry.addData("Heading", heading);
        telemetry.addLine("==========================================");
    }

    // Auto Aiming
    final double shooterOffset = 3.5; //inches

    private int getHive() {
        if (y < 60){
            return 0;
        }
        else if (y > 80){
            return 1;
        } else return -1;
    }
    private double getHiveAngle() { //returns angle in degrees
        double hd = Math.toDegrees(heading);
        double dx = (58) - x - (Math.cos(Math.toRadians(hd)) * shooterOffset);
        double dy = (53 + (37 * getHive())) - y - (Math.sin(Math.toRadians(hd)) * shooterOffset);
        dy -= Math.signum(dy) * Math.abs(dx/25);
        double da = Math.toDegrees(Math.atan2(dy,dx));
        if (hd - da < 180) {
            return (hd - da);
        } else {
            return (-360 + (hd - da));
        }
    }
    private double getAngleError() {
        return (getTurretPosition() - getHiveAngle());
    }
    private double getHiveDistance() { //returns distance in metres
        double hd = Math.toDegrees(heading);
        double dx = (58) - x - (Math.cos(Math.toRadians(hd)) * shooterOffset);
        double dy = (53 + (37 * getHive())) - y - (Math.sin(Math.toRadians(hd)) * shooterOffset);
        dy -= Math.signum(dy) * Math.abs(dx/25);
        return (Math.hypot(dy, dx)) / 39.37;
    }
    
    // Shooter (Flywheel, Hood, Transfer and Turret functions combined)
    boolean moving;
    private void shooterIdle() {
        setFlywheelVelocity(idleFlywheelVelocity);
        setHoodPosition(idleHoodPosition);
        setTurretPosition(idleTurretPosition);
        setTransferPower(idleTransferPower);
    }
    private void shooterActive() {
        aimTurret();
        setDynamicFlywheelVelocity();
        setDynamicHoodPosition();
    }
    public void commandShooterActive() {
        shooterActive();
        setTransferPower(activeTransferPower);
    }
    public void shooterControl() {
        if (gamepad1.leftTriggerWasPressed()) {
            setTransferPower(activeTransferPower);
        } else if (!gamepad1.left_trigger_pressed){
            shooterIdle();
        }
        if (!follower.holding()) {
            moving = true;
        }
        if (moving && gamepad1.left_trigger_pressed){
            shooterActive();
        } else if (moving && gamepad1.left_trigger_pressed && follower.holding()) {
            moving = false;
            shooterActive();
        }
    }

    public void shooterTelemetry() {
        telemetry.addLine("Shooter Controls: ");
        telemetry.addLine("Initiate Shooting - Hold Left Trigger");
        telemetry.addLine("Shoot - Hold Right Trigger");
        telemetry.addData("Hive Angle", getHiveAngle());
        telemetry.addLine("==========================================");
    }
}
