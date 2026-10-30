package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@TeleOp(name="Calibration TeleOp")
public class CalibrationTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Calibration cal = new Calibration(hardwareMap, gamepad1, gamepad2, telemetry);
        waitForStart();
        while (opModeIsActive())
        {
            cal.varsUpdate();
            cal.resetOdometry();
            cal.getStickAngle();
            cal.stickAngleTelemetry();
            //!!!tuneShooter(), tuneFlywheelPIDF(), cycleTurretMode(), and RTCdriveUpdate() are INCOMPATIBLE with eachother (use one at a time)!!!
            //cal.tuneShooter();
            //cal.tuneFlywheelPIDF();
            cal.cycleTurretMode();
            //cal.RCTdriveUpdate();
            //cal.tuneHoodPosition();
            cal.toggleBlocker();
            cal.toggleIntake();
            cal.toggleTransfer();
            cal.toggleDepollinator();
            telemetry.update();
        }
    }
}