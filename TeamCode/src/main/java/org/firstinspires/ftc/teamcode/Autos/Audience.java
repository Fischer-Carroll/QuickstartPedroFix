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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@Disabled
@Autonomous(name = "Audience", group = "Autonomous")
public class Audience extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.25, 133.25, 90);
    private final Pose scoringFlower2 = poseFactory.of(48, 122, 90);
    private final Pose scoringFlower2Control1 = poseFactory.of(60, 126, 0);
    private final Pose scoringFlower2Control2 = poseFactory.of(56, 122, 0);
    private final Pose flower2Shimmy = poseFactory.of(48, 128, 90);
    private final Pose point3 = poseFactory.of(12, 120, 0);
    private final Pose point3Control1 = poseFactory.of(48, 120, 0);
    private final Pose point3Segment1Heading = poseFactory.of(12, 120, 90);
    private final Pose point3Segment2Start = poseFactory.of(12, 120, 90);
    private final Pose point3Segment2End = poseFactory.of(12, 120, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, scoringFlower2()),
                follow(follower, flower2Shimmy()),
                follow(follower, path3())
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

    public Path scoringFlower2() {
        return Paths.curve(start, scoringFlower2Control1, scoringFlower2Control2, scoringFlower2).constant(scoringFlower2);
    }

    public Path flower2Shimmy() {
        return Paths.line(scoringFlower2, flower2Shimmy).constant(flower2Shimmy);
    }

    public Path path3() {
        return Paths.curve(flower2Shimmy, point3Control1, point3).heading(Interpolator.piecewise().until(0.2154, Interpolator.constant(point3Segment1Heading)).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }
}