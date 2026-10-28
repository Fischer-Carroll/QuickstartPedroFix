package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "ScoringAuto", group = "Autonomous")
public class ScoringAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.25, 8.25, 90);
    private final Pose startShoot = poseFactory.of(55.25, 8.25, 180);
    private final Pose startFlower1 = poseFactory.of(15, 48, 180);
    private final Pose startFlower1Control1 = poseFactory.of(33.1139, 47.964, 0);
    private final Pose startFlower1Segment1Start = poseFactory.of(15, 48, 180);
    private final Pose startFlower1Segment1End = poseFactory.of(15, 48, 180);
    private final Pose startFlower1Segment2Start = poseFactory.of(15, 48, 180);
    private final Pose startFlower1Segment2End = poseFactory.of(15, 48, 127);
    private final Pose startFlower1Segment4Start = poseFactory.of(15, 48, 160);
    private final Pose startFlower1Segment4End = poseFactory.of(15, 48, 180);
    private final Pose startFlower1Segment5Heading = poseFactory.of(15, 48, 180);
    private final Pose flower1Hive1 = poseFactory.of(52, 9.25, 180);
    private final Pose flower1Hive1Segment1Start = poseFactory.of(52, 9.25, 180);
    private final Pose flower1Hive1Segment1End = poseFactory.of(52, 9.25, 180);
    private final Pose flower1Hive1Segment2Start = poseFactory.of(52, 9.25, 180);
    private final Pose flower1Hive1Segment2End = poseFactory.of(52, 9.25, 136);
    private final Pose flower1Hive1Segment4Start = poseFactory.of(52, 9.25, 133.6766);
    private final Pose flower1Hive1Segment4End = poseFactory.of(52, 9.25, 180);
    private final Pose hive1Garden = poseFactory.of(9.256, 9.25, 180);
    private final Pose gardenHive1 = poseFactory.of(52, 12, 304);
    private final Pose gardenHive1Control1 = poseFactory.of(13.5963, 16.1361, 0);
    private final Pose gardenHive1Segment1Start = poseFactory.of(52, 12, 180);
    private final Pose gardenHive1Segment1End = poseFactory.of(52, 12, -56);
    private final Pose point6 = poseFactory.of(12, 96, -89.5261);
    private final Pose point6Control1 = poseFactory.of(12, 72, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startShoot()),
                follow(follower, startFlower1()),
                follow(follower, flower1Hive1()),
                follow(follower, hive1Garden()),
                follow(follower, gardenHive1()),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
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
    }

    public Path startShoot() {
        return Paths.line(start, startShoot).constant(startShoot);
    }

    public Path startFlower1() {
        return Paths.curve(startShoot, startFlower1Control1, startFlower1).heading(Interpolator.piecewise().until(0.0348, Interpolator.linear(startFlower1Segment1Start, startFlower1Segment1End)).until(0.1431, Interpolator.linear(startFlower1Segment2Start, startFlower1Segment2End)).until(0.8279, Interpolator.tangent).until(0.9139, Interpolator.linear(startFlower1Segment4Start, startFlower1Segment4End)).until(1, Interpolator.constant(startFlower1Segment5Heading)));
    }

    public Path flower1Hive1() {
        return Paths.line(startFlower1, flower1Hive1).heading(Interpolator.piecewise().until(0.0861, Interpolator.linear(flower1Hive1Segment1Start, flower1Hive1Segment1End)).until(0.3443, Interpolator.linear(flower1Hive1Segment2Start, flower1Hive1Segment2End)).until(0.6721, Interpolator.tangent.reverse()).until(1, Interpolator.linear(flower1Hive1Segment4Start, flower1Hive1Segment4End)));
    }

    public Path hive1Garden() {
        return Paths.line(flower1Hive1, hive1Garden).constant(hive1Garden);
    }

    public Path gardenHive1() {
        return Paths.curve(hive1Garden, gardenHive1Control1, gardenHive1).heading(Interpolator.piecewise().until(1, Interpolator.linear(gardenHive1Segment1Start, gardenHive1Segment1End)));
    }

    public Path path6() {
        return Paths.curve(gardenHive1, point6Control1, point6).heading(Interpolator.piecewise().until(1, Interpolator.tangent.reverse()));
    }
}