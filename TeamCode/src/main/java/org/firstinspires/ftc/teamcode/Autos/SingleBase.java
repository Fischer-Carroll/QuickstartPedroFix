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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class SingleBase extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60, 132, 90);
    private final Pose startFlower2 = poseFactory.of(46.75, 125.75, 90);
    private final Pose startFlower2Control1 = poseFactory.of(59.75, 107.75, 0);
    private final Pose startFlower2Control2 = poseFactory.of(47.75, 107.75, 0);
    private final Pose startFlower2Segment1Start = poseFactory.of(46.75, 125.75, 270);
    private final Pose startFlower2Segment1End = poseFactory.of(46.75, 125.75, 90);
    private final Pose startFlower2Segment2Heading = poseFactory.of(46.75, 125.75, 90);
    private final Pose flower2Hive2 = poseFactory.of(59.75, 128.75, 0);
    private final Pose flower2Hive2Control1 = poseFactory.of(47.75, 107.75, 0);
    private final Pose flower2Hive2Control2 = poseFactory.of(59.75, 107.75, 0);
    private final Pose flower2Hive2Segment1Start = poseFactory.of(59.75, 128.75, 90);
    private final Pose flower2Hive2Segment1End = poseFactory.of(59.75, 128.75, 0);
    private final Pose hive2Base = poseFactory.of(9.75, 118.75, 0);
    private final Pose hive2BaseControl1 = poseFactory.of(59.75, 107.75, 0);
    private final Pose hive2BaseControl2 = poseFactory.of(10.75, 141.5, 0);
    private final Pose hive2BaseControl3 = poseFactory.of(10.75, 137.75, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startFlower2()), //before moving, should aim and shoot into hive and lower depollinator and turn on intake
                follow(follower, flower2Hive2()), //after, should turn off intake and aim and shoot into hive and raise depollinator
                follow(follower, hive2Base())
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
        return Paths.curve(start, startFlower2Control1, startFlower2Control2, startFlower2).heading(Interpolator.piecewise().until(0.7223, Interpolator.linear(startFlower2Segment1Start, startFlower2Segment1End)).until(1, Interpolator.constant(startFlower2Segment2Heading)));
    }

    public Path flower2Hive2() {
        return Paths.curve(startFlower2, flower2Hive2Control1, flower2Hive2Control2, flower2Hive2).heading(Interpolator.piecewise().until(1, Interpolator.linear(flower2Hive2Segment1Start, flower2Hive2Segment1End)));
    }

    public Path hive2Base() {
        return Paths.curve(flower2Hive2, hive2BaseControl1, hive2BaseControl2, hive2BaseControl3, hive2Base).constant(hive2Base);
    }
}