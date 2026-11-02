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

@Autonomous(name = "Single Garden", group = "Autonomous")
public class SingleAudiencePark extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 7.75, 90);
    private final Pose startGarden = poseFactory.of(10, 9, 180);
    private final Pose gardenHive1 = poseFactory.of(57, 9, 180);
    private final Pose hive1Park = poseFactory.of(9.75, 94.75, 0);
    private final Pose hive1ParkControl1 = poseFactory.of(14.75, 9.75, 0);
    private final Pose hive1ParkControl2 = poseFactory.of(8.75, 66.75, 0);
    private final Pose hive1ParkControl3 = poseFactory.of(26.75, 56.75, 0);
    private final Pose hive1ParkSegment2Start = poseFactory.of(9.75, 94.75, 103.8632);
    private final Pose hive1ParkSegment2End = poseFactory.of(9.75, 94.75, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startGarden()), // before moving, should aim and shoot into hive and turn intake on
                follow(follower, gardenHive1()), //after, should aim and shoot into hive and turn intake off
                follow(follower, hive1Park()) //end
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

    public Path startGarden() {
        return Paths.line(start, startGarden).constant(startGarden);
    }

    public Path gardenHive1() {
        return Paths.line(startGarden, gardenHive1).constant(gardenHive1);
    }

    public Path hive1Park() {
        return Paths.curve(gardenHive1, hive1ParkControl1, hive1ParkControl2, hive1ParkControl3, hive1Park).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1, Interpolator.linear(hive1ParkSegment2Start, hive1ParkSegment2End)));
    }
}