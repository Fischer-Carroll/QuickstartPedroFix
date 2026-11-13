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
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.LM1Robot;

@Autonomous(name = "Single Base Auto", group = "Autonomous")
public class SingleBase extends LinearOpMode {

    private Follower f;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.25, 136.25, 270);
    private final Pose startFlower2 = poseFactory.of(46.75, 124.75, 90);
    private final Pose startFlower2Control1 = poseFactory.of(58.9428, 108.75, 0);
    private final Pose startFlower2Control2 = poseFactory.of(46.75, 108.75, 0);
    private final Pose startFlower2Segment1Start = poseFactory.of(46.75, 124.75, 270);
    private final Pose startFlower2Segment1End = poseFactory.of(46.75, 124.75, 90);
    private final Pose startFlower2Segment2Heading = poseFactory.of(46.75, 124.75, 90);
    private final Pose flower2Hive2Start = poseFactory.of(46.75, 124.75, 90);
    private final Pose flower2Hive2 = poseFactory.of(55.75, 124.75, 10);
    private final Pose flower2Hive2Control1 = poseFactory.of(45.75, 114.75, 0);
    private final Pose flower2Hive2Control2 = poseFactory.of(54.75, 114.75, 0);
    private final Pose hive2Wait = poseFactory.of(12.75, 132.75, 0);
    private final Pose hive2WaitControl1 = poseFactory.of(35.75, 114.75, 0);
    private final Pose hive2WaitControl2 = poseFactory.of(13.75, 138.75, 0);
    private final Pose hive2WaitControl3 = poseFactory.of(17.75, 130.75, 0);
    private final Pose hive2WaitSegment1Start = poseFactory.of(12.75, 132.75, 10);
    private final Pose hive2WaitSegment1End = poseFactory.of(12.75, 132.75, 0);
    private final Pose hive2WaitSegment2Heading = poseFactory.of(12.75, 132.75, 0);
    private final Pose waitBase = poseFactory.of(12.75, 119.75, 0);
    LM1Robot bot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                waitMs(15000),
                bot.cmdStart,
                bot.cmdBlockerDown,
                waitMs(250),
                bot.cmdIdleShooter,
                bot.cmdIntake,
                waitMs(100),
                bot.cmdDepollinatorDown,
                bot.cmdShoot,
                follow(f, startFlower2()),
                waitMs(3000),
                follow(f, flower2Hive2()),
                bot.cmdShoot,
                bot.cmdNulltake,
                follow(f, hive2Wait()),
                bot.cmdStop,
                waitMs(4000),
                follow(f, waitBase())
        );
    }

    @Override
    public void runOpMode() {
        bot = new LM1Robot(hardwareMap,gamepad1,gamepad2,telemetry);
        bot.commands();
        Scheduler.reset();
        f = bot.follower;
        f.setPose(start);
        f.update();

        waitForStart();
        bot.start();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            bot.positionUpdate();
            f.update();
            Scheduler.execute();

            bot.depollinatorJiggle();
            telemetry.addData("x", f.pose().x());
            telemetry.addData("y", f.pose().y());
            telemetry.addData("heading", f.pose().heading());

            if (f.currentPath() != null) {
                telemetry.addData("Current path distance remaining", f.distanceToEndpoint());
                telemetry.addData("Path number", f.pathIndex());
            }

            telemetry.update();
        }
        bot.stop();
    }

    public Path startFlower2() {
        return Paths.curve(start, startFlower2Control1, startFlower2Control2, startFlower2).heading(Interpolator.piecewise().until(0.7315, Interpolator.linear(startFlower2Segment1Start, startFlower2Segment1End)).until(1, Interpolator.constant(startFlower2Segment2Heading)));
    }

    public Path flower2Hive2() {
        return Paths.curve(flower2Hive2Start, flower2Hive2Control1, flower2Hive2Control2, flower2Hive2).linear(flower2Hive2Start, flower2Hive2);
    }

    public Path hive2Wait() {
        return Paths.curve(flower2Hive2, hive2WaitControl1, hive2WaitControl2, hive2WaitControl3, hive2Wait).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(hive2WaitSegment1Start, hive2WaitSegment1End)).until(1, Interpolator.constant(hive2WaitSegment2Heading)));
    }

    public Path waitBase() {
        return Paths.line(hive2Wait, waitBase).constant(waitBase);
    }
}
