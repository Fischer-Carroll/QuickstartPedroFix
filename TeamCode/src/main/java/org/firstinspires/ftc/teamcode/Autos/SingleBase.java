package org.firstinspires.ftc.teamcode.Autos;

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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Single Base", group = "Autonomous")
public class SingleBase extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60, 134, 180);
    private final Pose startFlower2 = poseFactory.of(46.75, 124.75, 90);
    private final Pose startFlower2Control1 = poseFactory.of(60.75, 108.75, 0);
    private final Pose startFlower2Control2 = poseFactory.of(46.75, 108.75, 0);
    private final Pose startFlower2Segment1Start = poseFactory.of(46.75, 124.75, 270);
    private final Pose startFlower2Segment1End = poseFactory.of(46.75, 124.75, 90);
    private final Pose startFlower2Segment2Heading = poseFactory.of(46.75, 124.75, 90);
    private final Pose flower2Hive2Start = poseFactory.of(46.75, 124.75, 90);
    private final Pose flower2Hive2 = poseFactory.of(55.75, 124.75, 35);
    private final Pose flower2Hive2Control1 = poseFactory.of(45.75, 114.75, 0);
    private final Pose flower2Hive2Control2 = poseFactory.of(54.75, 114.75, 0);
    private final Pose hive2Base = poseFactory.of(12.75, 132.75, 0);
    private final Pose hive2BaseControl1 = poseFactory.of(35.75, 114.75, 0);
    private final Pose hive2BaseControl2 = poseFactory.of(13.75, 138.75, 0);
    private final Pose hive2BaseControl3 = poseFactory.of(17.75, 130.75, 0);
    private final Pose hive2BaseSegment1Start = poseFactory.of(12.75, 132.75, 35);
    private final Pose hive2BaseSegment1End = poseFactory.of(12.75, 132.75, 0);
    private final Pose hive2BaseSegment2Heading = poseFactory.of(12.75, 132.75, 0);
    private final Pose point4 = poseFactory.of(12.75, 119.75, 0);
    Robot b;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                b.cmdPrimeTurret,
                waitMs(2000),
                b.cmdShoot,
                waitMs(500),
                follow(follower, startFlower2()), //before moving, should aim and shoot into hive and lower depollinator and turn on intake
                follow(follower, flower2Hive2()), //after, should turn off intake and aim and shoot into hive and raise depollinator
                follow(follower, hive2Base()),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        b = new Robot(hardwareMap, gamepad1,telemetry);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        b.closeBlocker();
        b.initialiseServoScales();
        waitForStart();
        b.initialiseActuatorPositions();
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
        return Paths.curve(start, startFlower2Control1, startFlower2Control2, startFlower2).heading(Interpolator.piecewise().until(0.7315, Interpolator.linear(startFlower2Segment1Start, startFlower2Segment1End)).until(1, Interpolator.constant(startFlower2Segment2Heading)));
    }

    public Path flower2Hive2() {
        return Paths.curve(flower2Hive2Start, flower2Hive2Control1, flower2Hive2Control2, flower2Hive2).linear(flower2Hive2Start, flower2Hive2);
    }

    public Path hive2Base() {
        return Paths.curve(flower2Hive2, hive2BaseControl1, hive2BaseControl2, hive2BaseControl3, hive2Base).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(hive2BaseSegment1Start, hive2BaseSegment1End)).until(1, Interpolator.constant(hive2BaseSegment2Heading)));
    }

    public Path path4() {
        return Paths.line(hive2Base, point4).constant(point4);
    }
}