/*package org.firstinspires.ftc.teamcode.Autos;

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
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.25, 8.25, 90);
    private final Pose startGarden = poseFactory.of(9.25, 9.25, 180);
    private final Pose gardenFlower2 = poseFactory.of(62, 136, 0);
    private final Pose gardenFlower2Control1 = poseFactory.of(27.3913, 32.7332, 0);
    private final Pose gardenFlower2Control2 = poseFactory.of(18.1533, 126.021, 0);
    private final Pose gardenFlower2Control3 = poseFactory.of(48.271, 98.9371, 0);
    private final Pose gardenFlower2Segment1Start = poseFactory.of(62, 136, 180);
    private final Pose gardenFlower2Segment1End = poseFactory.of(62, 136, 180);
    private final Pose gardenFlower2Segment2Start = poseFactory.of(62, 136, 180);
    private final Pose gardenFlower2Segment2End = poseFactory.of(62, 136, 75);
    private final Pose gardenFlower2Segment4Start = poseFactory.of(62, 136, 44.674);
    private final Pose gardenFlower2Segment4End = poseFactory.of(62, 136, 0);
    private final Pose flower2Shoot2Start = poseFactory.of(62, 136, 0);
    private final Pose flower2Shoot2 = poseFactory.of(48, 132, 90);
    private final Pose flower2Shoot2Control1 = poseFactory.of(61.0946, 114.4431, 0);
    private final Pose flower2Shoot2Control2 = poseFactory.of(48.2817, 99.0244, 0);
    private final Pose flower2Hive2 = poseFactory.of(62, 136, 0);
    private final Pose flower2Hive2Control1 = poseFactory.of(48, 99.1348, 0);
    private final Pose flower2Hive2Control2 = poseFactory.of(61.1233, 114.2688, 0);
    private final Pose flower2Hive2Segment1Heading = poseFactory.of(62, 136, 90);
    private final Pose flower2Hive2Segment2Start = poseFactory.of(62, 136, 90);
    private final Pose flower2Hive2Segment2End = poseFactory.of(62, 136, 0);
    private final Pose flower2APStart = poseFactory.of(62, 136, 0);
    private final Pose flower2AP = poseFactory.of(9.25, 86.25, 90);
    private final Pose flower2APControl1 = poseFactory.of(59.2228, 106.2591, 0);
    private final Pose flower2APControl2 = poseFactory.of(-0.1678, 119.6262, 0);
    private final Pose flower2APControl3 = poseFactory.of(52.4454, 49.8427, 0);
    private final Pose flower2APControl4 = poseFactory.of(8.2616, 55.6207, 0);
    private final Pose aPHive1 = poseFactory.of(62, 136, 0);
    private final Pose aPHive1Control1 = poseFactory.of(21.0269, 58.5337, 0);
    private final Pose aPHive1Segment1Heading = poseFactory.of(62, 136, 90);
    private final Pose aPHive1Segment2Start = poseFactory.of(62, 136, 90);
    private final Pose aPHive1Segment2End = poseFactory.of(62, 136, 0);
    private final Pose point7 = poseFactory.of(15, 48, 180);
    private final Pose point7Control1 = poseFactory.of(53.0982, 95.776, 0);
    private final Pose point7Control2 = poseFactory.of(6.7262, 115.3816, 0);
    private final Pose point7Control3 = poseFactory.of(36.7949, 50.4486, 0);
    private final Pose point7Segment1Start = poseFactory.of(15, 48, 0);
    private final Pose point7Segment1End = poseFactory.of(15, 48, 180);
    private final Pose point8Start = poseFactory.of(15, 48, 180);
    private final Pose point8 = poseFactory.of(56, 12, 0);
    private final Pose point9 = poseFactory.of(12, 120, 0);
    private final Pose point9Control1 = poseFactory.of(5.2374, 36.3771, 0);
    private final Pose point9Control2 = poseFactory.of(49.0791, 140.7054, 0);
    private final Pose point9Segment1Start = poseFactory.of(12, 120, 0);
    private final Pose point9Segment1End = poseFactory.of(12, 120, 108);
    private final Pose point9Segment3Start = poseFactory.of(12, 120, 90.1734);
    private final Pose point9Segment3End = poseFactory.of(12, 120, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startGarden()),
                follow(follower, gardenFlower2()),
                follow(follower, flower2Shoot2()),
                follow(follower, flower2Hive2()),
                follow(follower, flower2AP()),
                follow(follower, aPHive1()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9())
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

    public Path gardenFlower2() {
        return Paths.curve(startGarden, gardenFlower2Control1, gardenFlower2Control2, gardenFlower2Control3, gardenFlower2).heading(Interpolator.piecewise().until(0.1332, Interpolator.linear(gardenFlower2Segment1Start, gardenFlower2Segment1End)).until(0.4204, Interpolator.linear(gardenFlower2Segment2Start, gardenFlower2Segment2End)).until(0.8085, Interpolator.tangent).until(1, Interpolator.linear(gardenFlower2Segment4Start, gardenFlower2Segment4End)));
    }

    public Path flower2Shoot2() {
        return Paths.curve(flower2Shoot2Start, flower2Shoot2Control1, flower2Shoot2Control2, flower2Shoot2).linear(flower2Shoot2Start, flower2Shoot2);
    }

    public Path flower2Hive2() {
        return Paths.curve(flower2Shoot2, flower2Hive2Control1, flower2Hive2Control2, flower2Hive2).heading(Interpolator.piecewise().until(0.4963, Interpolator.constant(flower2Hive2Segment1Heading)).until(1, Interpolator.linear(flower2Hive2Segment2Start, flower2Hive2Segment2End)));
    }

    public Path flower2AP() {
        return Paths.curve(flower2APStart, flower2APControl1, flower2APControl2, flower2APControl3, flower2APControl4, flower2AP).linear(flower2APStart, flower2AP);
    }

    public Path aPHive1() {
        return Paths.curve(flower2AP, aPHive1Control1, aPHive1).heading(Interpolator.piecewise().until(0.2874, Interpolator.constant(aPHive1Segment1Heading)).until(1, Interpolator.linear(aPHive1Segment2Start, aPHive1Segment2End)));
    }

    public Path path7() {
        return Paths.curve(aPHive1, point7Control1, point7Control2, point7Control3, point7).heading(Interpolator.piecewise().until(1, Interpolator.linear(point7Segment1Start, point7Segment1End)));
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9Control2, point9).heading(Interpolator.piecewise().until(0.2323, Interpolator.linear(point9Segment1Start, point9Segment1End)).until(0.5894, Interpolator.tangent).until(1, Interpolator.linear(point9Segment3Start, point9Segment3End)));
    }*/