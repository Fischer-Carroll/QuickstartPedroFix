package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.utils.Timer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "smTest", group = "Autonomous")
public class smTest extends LinearOpMode {
    private Follower follower;

    private Robot bot;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.25, 8.25, 180);
    private final Pose startLeave = poseFactory.of(55.24, 12, 180);

    private Path startLeave() {
        return line(start,startLeave).linear(start,startLeave);
    }
    // Autonomous routine
    public CommandBuilder autoRoutine() {
        return sequential(
                bot.cmdCloseBlocker,
                waitMs(2500),
                bot.shoot,
                waitMs(2000),
                follow(follower, startLeave())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        bot = new Robot(hardwareMap, gamepad1, telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        waitForStart();

        schedule(autoRoutine());

        bot.sethoodPosition(0.75);
        bot.transfer.setPower(1);
        bot.intake.setPower(1);
        bot.setFlywheelTPS(2140);
        bot.closeBlocker();

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());

            telemetry.update();
        }
        OpModeStorage.autonomousEndPose = follower.pose();
    }
}
