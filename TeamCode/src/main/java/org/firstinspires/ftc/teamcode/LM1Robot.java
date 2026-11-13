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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.utils.Utils.lerp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class LM1Robot {
    // Miscellaneous
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;

    public Follower follower;

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
    VoltageSensor voltageSensor;

    // Timers
    Timer runtime;
    Timer depollinatorTimer;
    Timer takerTimer;

    public LM1Robot(HardwareMap hwMap, Gamepad gp1, Gamepad gp2, Telemetry tlm) {
        // Passed Objects
        hardwareMap = hwMap;
        gamepad1 = gp1;
        gamepad2 = gp2;
        telemetry = tlm;

        // Hardware Map
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        taker = hardwareMap.get(DcMotorEx.class, "taker");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depollinator = hardwareMap.get(Servo.class, "depollinator");

        // Configurations
        follower = Constants.create(hardwareMap);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo Scales
        turret1.scaleRange(0.025,0.975);
        turret2.scaleRange(0.025,0.975);
        hood.scaleRange(0.2,0.625);
        blocker.scaleRange(0.21,0.4);
        depollinator.scaleRange(0.232,0.57);
    }

    public void start() {
        // Timers
        runtime = new Timer();
        depollinatorTimer = new Timer();
        takerTimer = new Timer();
        
        // Mechanisms
        depollinatorUp();
        targetTakerPower = 1;
        intake();
        shooterIdle();
    }
    public void cmdStart() {
        runtime = new Timer();
        depollinatorTimer = new Timer();
        takerTimer = new Timer();

        depollinatorUp();
        targetTakerPower = 0;
        nulltake();
    }
    public void cmdStop() {
        depollinatorUp();
        setFlywheelVelocity(0);
        setHoodPosition(idleHoodPosition);
        setTurretPosition(idleTurretPosition);
        setTransferPower(0);
        nulltake();
    }

    // Commands
    public Command cmdStart;
    public Command cmdStop;

    public Command cmdBlockerUp;
    public Command cmdBlockerDown;

    public Command cmdActivateShooter;
    public Command cmdIdleShooter;
    public Command cmdShoot;

    public Command cmdDepollinatorUp;
    public Command cmdDepollinatorDown;

    public Command cmdIntake;
    public Command cmdNulltake;
    public Command cmdOuttake;

    public void commands() {
        cmdStart = Command.build()
                .setStart(this::cmdStart)
                .setDone(() -> true)
                .requiring(depollinator,taker,flywheel1,flywheel2,hood,transfer,turret1,turret2);
        cmdStop = Command.build()
                .setStart(this::cmdStop)
                .setDone(() -> true)
                .requiring(depollinator,taker,flywheel1,flywheel2,hood,transfer,turret1,turret2);


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
        cmdIdleShooter = Command.build()
                .setStart(this::commandShooterIdle)
                .setDone(() -> true)
                .requiring(turret1,turret2,transfer,flywheel1,flywheel2,hood);
        cmdShoot = sequential(
                cmdActivateShooter,
                waitMs(1200),
                cmdBlockerUp,
                waitMs(900),
                cmdBlockerDown,
                cmdIdleShooter
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
        cmdOuttake = Command.build()
                .setStart(this::outtake)
                .setDone(() -> true)
                .requiring(taker);
    }
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
    // Flywheel
    double flywheelVelocity;
    double targetFlywheelVelocity;
    double kP;
    double kF;
    double flywheelPower;
    double curFlywheelVelocity;
    int bbThreshold = 85;
    final double idleFlywheelVelocity = 2080;

    private void setFlywheelVelocity(double velocity) {
        flywheelVelocity = velocity;
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }
    private void setFlywheelPower(double power) {
        flywheelPower = power;
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }
    private void setTargetFlywheelVelocity() {
        double x = getHiveDistance();
        setFlywheelVelocity((1052*x*x*x*x - 4823*x*x*x + 7978*x*x - 5447*x + 3286) * 1.017);
    }
    private double getFlywheelVelocity() {
        return (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2;
    }
    private double getFlywheelKf() {
        return voltageSensor.getVoltage() / 2706.0;
    }
    private void flywheelBBFF() {
        curFlywheelVelocity = getFlywheelVelocity();
        if (curFlywheelVelocity < targetFlywheelVelocity - bbThreshold) {
            if (flywheelPower != 1) {setFlywheelPower(1);}
        } else if (curFlywheelVelocity > targetFlywheelVelocity + bbThreshold) {
            if (flywheelPower != 0) {setFlywheelPower(0);}
        } else {
            kP = (targetFlywheelVelocity - curFlywheelVelocity) * getFlywheelKf();
            kF = targetFlywheelVelocity * getFlywheelKf();
            setFlywheelPower((kP + kF) / voltageSensor.getVoltage());
        }
    }
    public void flywheelTelemetry() {
        curFlywheelVelocity = getFlywheelVelocity();
        telemetry.addData("Flywheel Velocity", curFlywheelVelocity);
        telemetry.addData("Target Flywheel Velocity", flywheelVelocity);
        telemetry.addData("Flywheel Velocity Error", (flywheelVelocity - curFlywheelVelocity));
    }

    // Hood
    double hoodPosition;
    final double idleHoodPosition = 0.75;

    private void setHoodPosition(double position) {
        hoodPosition = position;
        hood.setPosition(position);
    }
    private void setTargetHoodPosition(){
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

    // Taker (formerly referred to as the intake)
    double takerPower;
    double targetTakerPower;
    double initialTakerPower;
    double initialTargetTakerPower;
    double startLerpTime;
    double endLerpTime;
    final double outtakeTime = 0.5; //seconds
    final double intakePower = 1;
    final double nulltakePower = 0;
    final double outtakePower = -1;

    private void setTakerPower(double power) {
        takerPower = power;
        taker.setPower(power);
    }
    private void intake() {setTakerPower(intakePower);}
    private void outtake() {setTakerPower(outtakePower);}
    private void nulltake() {setTakerPower(nulltakePower);}

    public void takerControl() {
        if (gamepad1.xWasPressed()) { //turns off taker
            targetTakerPower = nulltakePower; //saves as this value for reference
            nulltake();
        }
        if (gamepad1.rightBumperWasPressed()) {
            targetTakerPower = intakePower;

            if (takerPower >= nulltakePower) {intake();} // if power is 0-1, immediately set the power to 1
            else { //if it's below 0 (outtaking), in order to not change rpm too fast
                initialTakerPower = takerPower; //saves current power as initial power to lerp from
                takerTimer.reset(); //resets timer so that it can be referenced from 0
            }
        }
        if (gamepad1.bWasPressed()) {
            if (targetTakerPower != outtakePower) {initialTargetTakerPower = targetTakerPower;} //stops from overriding initial target power if button is pressed multiple times
            initialTakerPower = takerPower; //saves so it is not overwritten
            targetTakerPower = outtakePower;
            takerTimer.reset();
        }
        if (targetTakerPower == intakePower && takerPower < intakePower) { //only if it is not already intakePower, to not be redundant
            setTakerPower(lerp(initialTakerPower, targetTakerPower, takerTimer.seconds() * 1.5)); //lerps power up to 1 from the initial power
        }
        if (targetTakerPower == outtakePower) { //lerps power down to outtake, then lerps back up to initial power
            startLerpTime = Math.abs(takerPower - targetTakerPower) / 2; //duration of the lerp, the 2 is arbitrary to lower the time it takes
            endLerpTime = Math.abs(targetTakerPower - initialTakerPower) / 2; //same, but for the return to the initial power
            if (takerTimer.seconds() < startLerpTime) { //while executing the initial lerp
                setTakerPower(lerp(initialTakerPower, targetTakerPower, takerTimer.seconds() / startLerpTime)); //scales the seconds from the timer by the lerp time to make it go 0-1
            }
            if (takerTimer.seconds() > startLerpTime + outtakeTime && takerTimer.seconds() < startLerpTime + outtakeTime + endLerpTime) { //if it has been outtaking for long enough and is not done lerping back
                setTakerPower(lerp(targetTakerPower, initialTargetTakerPower, (takerTimer.seconds() - startLerpTime - outtakeTime) / endLerpTime)); //subtracts elapsed time from the timer to still scale from 0-1
            }
            if (takerTimer.seconds() > startLerpTime + outtakeTime + endLerpTime) { //if it is done lerping
                targetTakerPower = initialTargetTakerPower; //passes power control back, afterwards ends the loop
                setTakerPower(targetTakerPower); //finalises the initial position
            }
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
            setDepollinatorPosition(depollinatorStates[depollinatorState % 2]);
        }
        depollinatorJiggle();
    }
    public void depollinatorTelemetry() {
        telemetry.addLine("Depollinator Controls: ");
        telemetry.addLine("Toggle Depollinator: Y");
        telemetry.addLine("==========================================");
    }

    // Drive
    public double l = 9.0;
    public double w = 7.75;
    double x;
    double y;
    double heading;
    public Pose odometryResetPose = new Pose(24 + l,w,0);

    public void positionUpdate() {
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
        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.addLine("==========================================");
    }

    // Auto Aiming
    final double shooterOffset = 3.5; //inches
    double hd;
    double dx;
    double dy;
    double da;
    double ax = 62;
    double ay = 55;

    private int getHive() {
        return (y < 72) ? 0 : 1;
    }
    public void tuneAimpointControl() {
        if (gamepad2.dpadRightWasPressed()) {
            ax -= 1;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            ax -= 1;
        }
        if (gamepad2.dpadUpWasPressed()) {
            ay += 1;
        }
        if (gamepad2.dpadDownWasPressed()) {
            ay -= 1;
        }
    }
    private void getAimpoint() {
        hd = Math.toDegrees(heading);
        dx = ax - x - (Math.cos(Math.toRadians(hd)) * shooterOffset);
        dy = (ay + ((90 - ay) * getHive())) - y - (Math.sin(Math.toRadians(hd)) * shooterOffset);
        dy += Math.signum(dy) * Math.abs(dx/20); //shifts aimpoint as x grows farther from hive
    }
    private double getHiveAngle() { //returns angle in degrees
        da = Math.toDegrees(Math.atan2(dy,dx));
        if (hd - da < 180) {
            return (hd - da);
        } else {
            return (-360 + (hd - da));
        }
    }
    private double getHiveDistance() { //returns distance in metres
        return (Math.hypot(dy, dx)) / 39.37;
    }
    
    // Shooter (Flywheel, Hood, Transfer and Turret functions combined)
    boolean moving;
    boolean active;
    private void shooterIdle() {
        if (active) {
            setFlywheelVelocity(idleFlywheelVelocity);
            setHoodPosition(idleHoodPosition);
            setTurretPosition(idleTurretPosition);
            setTransferPower(idleTransferPower);
            active = false;
        }
    }
    private void commandShooterIdle() {
        setFlywheelVelocity(idleFlywheelVelocity);
        setHoodPosition(idleHoodPosition);
        setTurretPosition(idleTurretPosition);
        setTransferPower(idleTransferPower);
    }
    private void shooterActive() {
        if (!active) {setTransferPower(activeTransferPower);}
        getAimpoint();
        aimTurret();
        setTargetFlywheelVelocity();
        setTargetHoodPosition();
        active = true;
    }
    private void commandShooterActive() {
        setTransferPower(activeTransferPower);
        getAimpoint();
        aimTurret();
        setTargetFlywheelVelocity();
        setTargetHoodPosition();
    }
    public void shooterControl() {
        if (!gamepad1.left_trigger_pressed){
            shooterIdle();
            gamepad1.stopRumble();
        }
        if (!follower.holding()) {
            moving = true;
        }
        if (moving && gamepad1.left_trigger_pressed){
            shooterActive();
        } else if (moving && gamepad1.left_trigger_pressed && follower.holding()) {
            moving = false;
            shooterActive();
            gamepad1.rumble(500);
        }
    }

    public void shooterTelemetry() {
        telemetry.addLine("Shooter Controls: ");
        telemetry.addLine("Initiate Shooting - Hold Left Trigger");
        telemetry.addLine("Shoot - Hold Right Trigger");
        telemetry.addData("Hive Angle", getHiveAngle());
        telemetry.addData("Flywheel Velocity (tps)", flywheel1.getVelocity());
        telemetry.addLine("==========================================");
    }
}
