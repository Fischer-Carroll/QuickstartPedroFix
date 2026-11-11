package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LM1Robot;
import org.firstinspires.ftc.teamcode.LM1TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Park Auto", group = "Autonomous")
public class Park extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.75, 133.75,  0);
    private final Pose startBase = poseFactory.of(12.75, 117.75, 0);
    private LM1Robot bot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startBase())
        );
    }

    @Override
    public void runOpMode() {
        bot = new LM1Robot(hardwareMap,gamepad1,gamepad2,telemetry);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        bot.stop();
    }

    public Path startBase() {
        return Paths.line(start, startBase).constant(startBase);
    }
}