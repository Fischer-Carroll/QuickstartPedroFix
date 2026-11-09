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

@TeleOp(name = "Testing")
public class Testing extends LinearOpMode {
    Robot b;
    double transferSpeed;
    int transferToggle;
    double flywheelUpper;
    double flywheelLower;
    @Override
    public void runOpMode() throws InterruptedException {
        b = new Robot(hardwareMap, gamepad1,telemetry);
        transferSpeed = 1;
        int transferToggle = 0;
        int flywheelSpeed = 2000;
        int intakeState = 0;
        b.initialiseServoScales();
        waitForStart();
        b.intake();
        while (opModeIsActive()){
            if (gamepad1.startWasPressed()){
                flywheelLower = flywheelSpeed;
                flywheelUpper = flywheelSpeed;
            }
            if (b.flywheel1.getVelocity() < flywheelLower){
                flywheelLower = b.flywheel1.getVelocity();
            }
            if (b.flywheel1.getVelocity() > flywheelUpper){
                flywheelUpper = b.flywheel1.getVelocity();
            }
            if (gamepad1.dpadUpWasPressed()){
                transferSpeed += 0.05;
                b.transfer.setPower(transferSpeed);
            }
            if (gamepad1.dpadDownWasPressed()){
                transferSpeed -= 0.05;
                b.transfer.setPower(transferSpeed);
            }
            if (gamepad1.dpadLeftWasPressed()){
                flywheelSpeed -= 10;
                b.setFlywheelTPS(flywheelSpeed);
            }
            if (gamepad1.dpadRightWasPressed()){
                flywheelSpeed += 10;
                b.setFlywheelTPS(flywheelSpeed);
            }
            if (gamepad1.leftBumperWasPressed()){
                intakeState = (intakeState+1)%2 - 1;
            }
            if (gamepad1.right_bumper){
                b.openBlocker();
            }else{
                b.closeBlocker();
            }
            if (gamepad1.aWasPressed()){
                transferToggle = (transferToggle+1)%2;
            }
            telemetry.addData("Flywheel Speed (TPS) ", flywheelSpeed);
            telemetry.addData("Transfer Speed (Power) ", transferSpeed);
            telemetry.addData("Highest flywheel Speed ", flywheelUpper);
            telemetry.addData("Lowest flywheel Speed ", flywheelLower);
            telemetry.addData("Transferring ", transferToggle);

            telemetry.update();
        }
    }

}
