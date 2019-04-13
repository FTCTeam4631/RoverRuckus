package org.team4631.ftc.teamcode.mission;

import com.dongbat.walkable.FloatArray;
import com.dongbat.walkable.PathHelper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.team4631.ftc.teamcode.controller.DriveController;
import org.team4631.ftc.teamcode.field.FieldMap;
import org.team4631.ftc.teamcode.hardware.HardwareRoss;
import org.team4631.ftc.teamcode.path.FirewiresPathPlanner;

import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Thread.sleep;

public class PathPlanningManager {

    private HardwareRoss hardwareRoss;
    private DriveController driveController;

    private FieldMap fieldMap;
    private PathHelper pathFinder;

    private ElapsedTime runtime = new ElapsedTime();

    public PathPlanningManager(HardwareRoss hardwareRoss) {
        this.hardwareRoss = hardwareRoss;
        this.driveController = new DriveController(hardwareRoss);

        this.fieldMap = new FieldMap();

        setupPathFinder();
    }

    public void followPath(float startX, float startY, float targetX, float targetY, double totalTime, double timeStep, int robotTrackWidth) {
        FloatArray path = new FloatArray();
        pathFinder.findPath(startX, startY, targetX, targetY, robotTrackWidth / 2, path);

        int points = path.size / 2;

        double[][] wayPoints = new double[points + 1][2];

        int j = 0;

        for (int i = 0; i < points * 2; i += 2) {
            wayPoints[j][0] = path.get(i);
            wayPoints[j][1] = 144 - path.get(i + 1);
            j++;
        }

        wayPoints[points][0] = targetX;
        wayPoints[points][1] = 144 - targetY;

        /*

        class FollowPathTask extends TimerTask {

            private double[][] wayPoints;
            private double totalTime;
            private double timeStep;
            private int robotTrackWidth;
            private FirewiresPathPlanner pathPlanner;
            private int currentTimeStepIndex;
            private int totalTimeSteps;

            public FollowPathTask(double[][] wayPoints, double totalTime, double timeStep, int robotTrackWidth) {
                this.wayPoints = wayPoints;
                this.totalTime = totalTime;
                this.timeStep = timeStep;
                this.robotTrackWidth = robotTrackWidth;
                this.pathPlanner = new FirewiresPathPlanner(wayPoints);
                this.pathPlanner.calculate(totalTime, timeStep, robotTrackWidth);
                this.currentTimeStepIndex = 0;
                this.totalTimeSteps = pathPlanner.smoothPath.length;
            }

            @Override
            public void run() {
                double leftV = pathPlanner.smoothLeftVelocity[currentTimeStepIndex][1];
                double rightV = pathPlanner.smoothRightVelocity[currentTimeStepIndex][1];

                driveController.setVelocitySetpoint(leftV, rightV, timeStep);

                hardwareRoss.clearTelemetry();
                hardwareRoss.addToTelemetry("Left velocity: ", leftV);
                hardwareRoss.addToTelemetry("Right velocity: ", rightV);
                hardwareRoss.updateTelemetry();

                currentTimeStepIndex++;

                if (currentTimeStepIndex >= totalTimeSteps) cancel();
            }

        }

        driveController.setDriveTrainIdle();

        FollowPathTask followPathTask = new FollowPathTask(wayPoints, totalTime, timeStep, robotTrackWidth);

        Timer timer = new Timer();
        timer.scheduleAtFixedRate(followPathTask, 0, (long)(timeStep * 1000));

        */

        FirewiresPathPlanner pathPlanner = new FirewiresPathPlanner(wayPoints);
        pathPlanner.calculate(totalTime, timeStep, robotTrackWidth);

        int currentTimeStepIndex = 0;
        int totalTimeSteps = pathPlanner.smoothPath.length;

        while (hardwareRoss.getLinearOpMode().opModeIsActive()) {
            double leftV = pathPlanner.smoothLeftVelocity[currentTimeStepIndex][1];
            double rightV = pathPlanner.smoothRightVelocity[currentTimeStepIndex][1];

            driveController.setVelocitySetpoint(leftV, rightV, timeStep);

            currentTimeStepIndex++;
            if (currentTimeStepIndex >= totalTimeSteps) break;

            try {
                sleep((long)(timeStep*1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void setupPathFinder() {
        pathFinder = new PathHelper((float)fieldMap.FULL_FIELD_LENGTH, (float)fieldMap.FULL_FIELD_LENGTH);

        float landerVertices[] = {
            72, 57,
            56, 72,
            72, 88,
            72, 37,
            88, 72,
            72, 88
        };

        float blueGoldDangerZoneVertices[] = {
            21, 96,
            24, 93,
            49, 119,
            46, 122
        };

        float blueSilverDangerZoneVertices[] = {
            19, 50,
            24, 55,
            52, 27,
            47, 22
        };

        float redGoldDangerZoneVertices[] = {
            92, 25,
            97, 20,
            126, 49,
            121, 54
        };

        float redSilverDangerZoneVertices[] = {
            91, 126,
            122, 95,
            126, 100,
            96, 131
        };

        pathFinder.addPolygon(landerVertices);
        pathFinder.addPolygon(blueGoldDangerZoneVertices);
        pathFinder.addPolygon(blueSilverDangerZoneVertices);
        pathFinder.addPolygon(redGoldDangerZoneVertices);
        pathFinder.addPolygon(redSilverDangerZoneVertices);
    }

}
