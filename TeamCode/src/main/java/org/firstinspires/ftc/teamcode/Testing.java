package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Drive Testing")
public class Testing extends LinearOpMode {
    Robot bot;
    double transferSpeed;
    int flywheelSpeed;
    int transferToggle;
    int flywheelUpper;
    int flywheelLower;
    @Override
    public void runOpMode() throws InterruptedException {
        b = new Robot(hardwareMap, gamepad1,telemetry);
        transferSpeed = 1;
        int transferToggle = 0;
        int flywheelSpeed;
        int intakeState = 0;
        bot.initialiseServoScales();
        waitForStart();
        
        while (opModeIsActive()){
            if (gamepad1.startWasPressed){
                flywheelLower = flywheelSpeed;
                flywheelUpper = flywheelSpeed;
            }
            if (b.bottomMotor.getVelocity() < flywheelLower){
                flywheelLower = bottomMotor.getVelocity();
            }
            if (b.bottomMotor.getVelocity() > flywheelUpper){
                flywheelUpper = bottomMotor.getVelocity();
            }
            if (gamepad1.dpadUpWasPressed){
                transferSpeed += 0.05;
                b.transfer.setPower(transferSpeed);
            }
            if (gamepad1.dpadDownWasPressed){
                transferSpeed -= 0.05;
                b.transfer.setPower(transferSpeed);
            }
            if (gamepad1.dpadLeftWasPressed){
                flywheelSpeed -= 10;
                b.setFlywheelTPS(flywheelSpeed);
            }
            if (gamepad1.dpadRightWasPressed){
                flywheelSpeed += 10;
                b.setFlywheelTPS(flywheelSpeed);
            }
            if (gamepad1.leftBumperWasPressed){
                intakeState = (intakeState+1)%2 - 1;
            }
            if (gamepad1.rightBumperWasPressed){
                b.openBlocker();
            }else{
                b.closeBlocekr();
            }
            telemetry.addData("Flywheel Speed (TPS) ", flywheelSpeed);
            telemetry.addData("Transfer Speed (Power) ", transferSpeed);
            telemetry.addData("Highest flywheel Speed ", flywheelUpper);
            telemetry.addData("Lowest flywheel Speed ", flywheelLower);
        }
    }
}
