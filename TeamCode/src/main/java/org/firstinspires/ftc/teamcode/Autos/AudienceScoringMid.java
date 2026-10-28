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

@Autonomous(name = "AudienceScoringMid", group = "Autonomous")
public class AudienceScoringMid extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.25, 133.25, 90);
    private final Pose startFlower2 = poseFactory.of(48, 122, 90);
    private final Pose startFlower2Control1 = poseFactory.of(60, 126, 0);
    private final Pose startFlower2Control2 = poseFactory.of(56, 122, 0);
    private final Pose flower2Shimmy = poseFactory.of(48, 128, 90);
    private final Pose flower2Flower1 = poseFactory.of(15, 48, 180);
    private final Pose flower2Flower1Control1 = poseFactory.of(48, 112, 0);
    private final Pose flower2Flower1Control2 = poseFactory.of(12, 120, 0);
    private final Pose flower2Flower1Control3 = poseFactory.of(48, 48, 0);
    private final Pose flower2Flower1Segment1Start = poseFactory.of(15, 48, 90);
    private final Pose flower2Flower1Segment1End = poseFactory.of(15, 48, 90);
    private final Pose flower2Flower1Segment2Start = poseFactory.of(15, 48, 90);
    private final Pose flower2Flower1Segment2End = poseFactory.of(15, 48, 49);
    private final Pose flower2Flower1Segment4Start = poseFactory.of(15, 48, 86.451);
    private final Pose flower2Flower1Segment4End = poseFactory.of(15, 48, 180);
    private final Pose flower2Flower1Segment5Start = poseFactory.of(15, 48, 180);
    private final Pose flower2Flower1Segment5End = poseFactory.of(15, 48, 180);
    private final Pose flower1Hive1Start = poseFactory.of(15, 48, 180);
    private final Pose flower1Hive1 = poseFactory.of(36, 14, 180);
    private final Pose hive1Garden = poseFactory.of(9.25, 9.25, 180);
    private final Pose hive1GardenControl1 = poseFactory.of(35.3318, 8.5411, 0);
    private final Pose gardenHive1 = poseFactory.of(36, 14, 271);
    private final Pose gardenHive1Segment1Start = poseFactory.of(36, 14, 180);
    private final Pose gardenHive1Segment1End = poseFactory.of(36, 14, 180);
    private final Pose gardenHive1Segment2Start = poseFactory.of(36, 14, 180);
    private final Pose gardenHive1Segment2End = poseFactory.of(36, 14, -89);
    private final Pose hive1Park = poseFactory.of(12, 120, -3.0447);
    private final Pose hive1ParkControl1 = poseFactory.of(33.9756, 119.36, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startFlower2()),
                follow(follower, flower2Shimmy()),
                follow(follower, flower2Flower1()),
                follow(follower, flower1Hive1()),
                follow(follower, hive1Garden()),
                follow(follower, gardenHive1()),
                follow(follower, hive1Park())
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

    public Path startFlower2() {
        return Paths.curve(start, startFlower2Control1, startFlower2Control2, startFlower2).linear(start, startFlower2);
    }

    public Path flower2Shimmy() {
        return Paths.line(startFlower2, flower2Shimmy).tangent();
    }

    public Path flower2Flower1() {
        return Paths.curve(flower2Shimmy, flower2Flower1Control1, flower2Flower1Control2, flower2Flower1Control3, flower2Flower1).heading(Interpolator.piecewise().until(0.0716, Interpolator.linear(flower2Flower1Segment1Start, flower2Flower1Segment1End)).until(0.1857, Interpolator.linear(flower2Flower1Segment2Start, flower2Flower1Segment2End)).until(0.6132, Interpolator.tangent.reverse()).until(0.9207, Interpolator.linear(flower2Flower1Segment4Start, flower2Flower1Segment4End)).until(1, Interpolator.linear(flower2Flower1Segment5Start, flower2Flower1Segment5End)));
    }

    public Path flower1Hive1() {
        return Paths.line(flower1Hive1Start, flower1Hive1).linear(flower1Hive1Start, flower1Hive1);
    }

    public Path hive1Garden() {
        return Paths.curve(flower1Hive1, hive1GardenControl1, hive1Garden).constant(hive1Garden);
    }

    public Path gardenHive1() {
        return Paths.line(hive1Garden, gardenHive1).heading(Interpolator.piecewise().until(0.279, Interpolator.linear(gardenHive1Segment1Start, gardenHive1Segment1End)).until(1, Interpolator.linear(gardenHive1Segment2Start, gardenHive1Segment2End)));
    }

    public Path hive1Park() {
        return Paths.curve(gardenHive1, hive1ParkControl1, hive1Park).reverseTangent();
    }
}