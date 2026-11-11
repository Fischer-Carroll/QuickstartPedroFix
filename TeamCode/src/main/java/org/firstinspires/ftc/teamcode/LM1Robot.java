package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

        // Servo Scales
        turret1.scaleRange(0.025,0.975);
        turret2.scaleRange(0.025,0.975);
        hood.scaleRange(0.2,0.625);
        blocker.scaleRange(0.21,0.4);
        depollinator.scaleRange(0.227,0.57);
    }

    public void start() {
        // Timers
        runtime = new Timer();
        depollinatorTimer = new Timer();
        takerTimer = new Timer();
        
        // Mechanisms
        depollinatorUp();
        targetTakerPower = 0;
        intake();
        shooterIdle();
    }
    public void cmdStart() {
        runtime = new Timer();
        depollinatorTimer = new Timer();
        takerTimer = new Timer();

        depollinatorUp();
        targetTakerPower = 0;
        intake();
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
                waitMs(900),
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
    final double idleFlywheelVelocity = 2080;

    private void setFlywheelVelocity(double velocity) {
        flywheelVelocity = velocity;
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }
    private void setDynamicFlywheelVelocity() {
        double x = getHiveDistance();
        setFlywheelVelocity((int)((1052*x*x*x*x - 4823*x*x*x + 7978*x*x - 5447*x + 3286) * 1.017));
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

    // Taker (formerly referred to as the intake)
    double takerPower;
    double targetTakerPower;
    double initialTakerPower;
    boolean initialTakerState;
    double startLerpTime;
    double endLerpTime;
    double outtakeTime = 0.75; //seconds
    boolean taking;
    boolean intake;
    boolean outtake;
    final double intakePower = 1;
    final double nulltakePower = 0;
    final double outtakePower = -1;

    private void setTakerPower(double power) {
        takerPower = power;
        taker.setPower(power);
    }
    private void intake() {setTakerPower(intakePower); intake = true;}
    private void outtake() {setTakerPower(outtakePower); outtake = true;}
    private void nulltake() {setTakerPower(0); taking = false;}
    public void takerControl() {
        if (gamepad1.xWasPressed()) {
            targetTakerPower = nulltakePower;
            takerTimer.reset();
        } else if (gamepad1.rightBumperWasPressed() && targetTakerPower != intakePower) {
            targetTakerPower = intakePower;
            takerTimer.reset();
        } else if (gamepad1.leftBumperWasPressed() && targetTakerPower != outtakePower) {
            targetTakerPower = outtakePower;
            takerTimer.reset();
        }
        if (takerTimer.seconds() < 1 && runtime.seconds() > 1) {
            setTakerPower(-targetTakerPower + (Math.signum(targetTakerPower) * takerTimer.seconds() * 4));
        }
    }
    public void takerControlV3() {
        if (gamepad1.xWasPressed() && taking) {
            intake = false;
            targetTakerPower = nulltakePower;
            nulltake();
        } else if (gamepad1.rightBumperWasPressed() && !intake) {
            taking = true;
            intake = true;
            takerTimer.reset();
            targetTakerPower = intakePower;
            if (takerPower == nulltakePower) {intake();}
        } else if (gamepad1.aWasPressed()) {
            initialTakerState = taking; //true if intake, false if nulltake
            taking = true;
            intake = false;
            outtake = true;
            takerTimer.reset();
            initialTakerPower = targetTakerPower;
            targetTakerPower = outtakePower;
        }
        if (targetTakerPower == intakePower && takerPower < intakePower && intake && taking) {
            startLerpTime = Math.abs(targetTakerPower - takerPower) / 4;
            setTakerPower(lerp(takerPower,targetTakerPower,takerTimer.seconds()));
        }
        if (targetTakerPower == outtakePower && takerTimer.seconds() < 1.75 && outtake && taking) {
            startLerpTime = Math.abs(targetTakerPower - takerPower) / 4;
            endLerpTime = Math.abs(targetTakerPower - initialTakerPower) / 4;
            if (takerTimer.seconds() < startLerpTime) {
                setTakerPower(lerp(takerPower,targetTakerPower,takerTimer.seconds()));
            }
            if (takerTimer.seconds() > startLerpTime + outtakeTime) {
                setTakerPower(lerp(targetTakerPower,initialTakerPower,takerTimer.seconds() - startLerpTime + outtakeTime));
            }
            if (takerTimer.seconds() > endLerpTime + startLerpTime + outtakeTime) {
                if (initialTakerState) {intake = true;}
                else {taking = false;}
                outtake = false;
            }
        }
    }

    public void takerControlV4() {
        if (gamepad1.xWasPressed()) {
            targetTakerPower = nulltakePower;
            nulltake();
        }
        if (gamepad1.rightBumperWasPressed()) {
            targetTakerPower = intakePower;

            if (takerPower >= 0) {intake();}
            else {
                initialTakerPower = takerPower;
                takerTimer.reset();
            }
        }
        if (gamepad1.aWasPressed()) {
            targetTakerPower = outtakePower;

            if (takerPower <= 0) {outtake();}
            else {
                initialTakerPower = takerPower;
                takerTimer.reset();
            }
        }
        if (targetTakerPower == intakePower && takerPower > intakePower) {
            setTakerPower(lerp(initialTakerPower, targetTakerPower, takerTimer.seconds() * 1.5));
        }
        if (targetTakerPower == outtakePower) {
            startLerpTime = Math.abs(takerPower - targetTakerPower) / 2;
            endLerpTime = Math.abs(targetTakerPower - initialTakerPower) / 2;
            if (takerTimer.seconds() < startLerpTime) {
                setTakerPower(lerp(initialTakerPower, targetTakerPower, takerTimer.seconds() / startLerpTime));
            }
            if (takerTimer.seconds() > startLerpTime + outtakeTime) {
                setTakerPower(lerp(targetTakerPower, initialTakerPower, takerTimer.seconds() / endLerpTime));
            }
            if (takerTimer.seconds() > startLerpTime + outtakeTime + endLerpTime) {
                targetTakerPower = initialTakerPower;
                setTakerPower(targetTakerPower);
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
    double ax = 58;
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
        setDynamicFlywheelVelocity();
        setDynamicHoodPosition();
        active = true;
    }
    private void commandShooterActive() {
        setTransferPower(activeTransferPower);
        getAimpoint();
        aimTurret();
        setDynamicFlywheelVelocity();
        setDynamicHoodPosition();
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
