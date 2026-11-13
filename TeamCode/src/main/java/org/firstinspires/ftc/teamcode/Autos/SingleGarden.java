package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.LM1Robot;

@Autonomous(name = "Single Garden Auto", group = "Autonomous")
public class SingleGarden extends LinearOpMode {

    private Follower f;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 7.75, 180);
    private final Pose startGarden = poseFactory.of(7.75, 10, -90);
    private final Pose startGardenControl1 = poseFactory.of(56.0985, 13.3534, 0);
    private final Pose startGardenControl2 = poseFactory.of(8.689, 38.2753, 0);
    private final Pose startGardenSegment1Start = poseFactory.of(7.75, 9, 180);
    private final Pose startGardenSegment1End = poseFactory.of(7.75, 9, 270);
    private final Pose startGardenSegment2Heading = poseFactory.of(7.75, 9, -90);
    private final Pose gardenHive1 = poseFactory.of(56.75, 11.75, -180);
    private final Pose gardenHive1Control1 = poseFactory.of(27.4977, 26.9615, 0);
    private final Pose gardenHive1Segment1Heading = poseFactory.of(56.75, 11.75, 270);
    private final Pose gardenHive1Segment2Start = poseFactory.of(56.75, 11.75, -90);
    private final Pose gardenHive1Segment2End = poseFactory.of(56.75, 11.75, 180);
    private final Pose hive1Park = poseFactory.of(14.75, 94.75, 0);
    private final Pose hive1ParkControl1 = poseFactory.of(14.75, 9.75, 0);
    private final Pose hive1ParkControl2 = poseFactory.of(12.75, 66.75, 0);
    private final Pose hive1ParkControl3 = poseFactory.of(14.75, 56.75, 0);
    private final Pose hive1ParkSegment1Heading = poseFactory.of(14.75, 94.75, 180);
    private final Pose hive1ParkSegment2Start = poseFactory.of(14.75, 94.75, 180);
    private final Pose hive1ParkSegment2End = poseFactory.of(14.75, 94.75, 120);
    private final Pose hive1ParkSegment4Start = poseFactory.of(14.75, 94.75, 97.6559);
    private final Pose hive1ParkSegment4End = poseFactory.of(14.75, 94.75, 0);
    LM1Robot bot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                bot.cmdStart,
                bot.cmdBlockerDown,
                waitMs(250),
                bot.cmdIdleShooter,
                bot.cmdIntake,
                waitMs(100),
                bot.cmdShoot,
                deadline(waitMs(6000),follow(f, startGarden())),
                follow(f, gardenHive1()),

        bot.cmdShoot,
                bot.cmdNulltake,
                follow(f, hive1Park()),
                bot.cmdStop
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

    public Path startGarden() {
        return Paths.curve(start, startGardenControl1, startGardenControl2, startGarden).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(startGardenSegment1Start, startGardenSegment1End)).until(1, Interpolator.constant(startGardenSegment2Heading)));
    }

    public Path gardenHive1() {
        return Paths.curve(startGarden, gardenHive1Control1, gardenHive1).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(gardenHive1Segment1Heading)).until(1, Interpolator.linear(gardenHive1Segment2Start, gardenHive1Segment2End)));
    }

    public Path hive1Park() {
        return Paths.curve(gardenHive1, hive1ParkControl1, hive1ParkControl2, hive1ParkControl3, hive1Park).heading(Interpolator.piecewise().until(0.1277, Interpolator.constant(hive1ParkSegment1Heading)).until(0.3257, Interpolator.linear(hive1ParkSegment2Start, hive1ParkSegment2End)).until(0.6297, Interpolator.tangent).until(1, Interpolator.linear(hive1ParkSegment4Start, hive1ParkSegment4End)));
    }
}
