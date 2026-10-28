package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private Servo blocker;
    private DcMotorEx bottomMotor;
    private DcMotorEx topMotor;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum TurretState
    {
        IDLE,
        LAUNCH,
    }
    private TurretState turretState;

    private int shootSpeed = 2140;
    private double shootAngle = 0.725;
    private double blockerOpen = 0;
    private double blockerClose = 1;
    private double shotTime = 0.3;
    private void init(HardwareMap hwMap){
        blocker = hwMap.get(Servo.class, "blocker");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");

        turretState = TurretState.IDLE;

        blocker.setPosition(blockerClose);
    }

    private void setFlywheelSpeed(int TPS){
        bottomMotor.setVelocity(TPS);
        topMotor.setVelocity(TPS);
    }

    public void update(){
        switch (turretState) {
            case IDLE:

                if (gamepad1.right_bumper){
                    stateTimer.reset();
                    turretState = TurretState.LAUNCH;
                }
                break;
            case LAUNCH:
                blocker.setPosition(blockerOpen);
                if (stateTimer.milliseconds() > 300)
                {
                    blocker.setPosition(blockerClose);
                    turretState = TurretState.IDLE;
                }
                break;

        }
    }
    public void launchPollen()
    {

    }
}
