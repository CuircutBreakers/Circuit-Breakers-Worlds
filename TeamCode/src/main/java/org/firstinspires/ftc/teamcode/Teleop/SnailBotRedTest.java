package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Snail Bot Red Test", group = "TeleOp")
public class SnailBotRedTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor spindle;
    private DcMotorEx launcherLeft, launcherRight;

    private Limelight3A limelight;

    private NormalizedColorSensor intakeSensorLeft, intakeSensorRight;
    private DistanceSensor intakeDistanceLeft, intakeDistanceRight;

    private final float[] leftHsv = new float[3];
    private final float[] rightHsv = new float[3];

    private int launcherDirection = 1;

    private static final double AIM_TARGET_TX = 0.0;
    private static final double FAR_AIM_TARGET_TX = -2.5;
    private static final double AIM_DEADBAND_DEG = 1.0;
    private static final double AIM_KP = 0.025;
    private static final double AUTO_AIM_TRIGGER_DEADBAND = 0.05;

    private static final double A1 = 0.5;
    private static final double A2 = 1.2;
    private static final double A3 = 1.8;
    private static final double A4 = 2.5;

    private static final double Ratio = 3.0 / 1.0;

    private static final double V_FAR  = 883 * Ratio;
    private static final double V_MID1 = 790 * Ratio;
    private static final double V_MID2 = 765 * Ratio;
    private static final double V_MID3 = 750 * Ratio;
    private static final double V_NEAR = 730 * Ratio;

    private static final double NO_TAG_BASE_VELOCITY = 750 * Ratio;
    private static final double VELOCITY_TRIM_STEP = 25;
    private double launchVelocityTrim = 0;

    private static final int SPINDLE_OPEN   = 140;
    private static final int SPINDLE_1BALL  = 50;
    private static final int SPINDLE_2BALL  = 10;

    private static final int SPINDLE_LAUNCH_1 = -75;
    private static final int SPINDLE_LAUNCH_2 = -250;
    private static final int SPINDLE_LAUNCH_3 = -340;

    private static final double SPINDLE_POWER = 1;

    private boolean lastA2 = false;
    private boolean lastB2 = false;
    private boolean lastX2 = false;
    private boolean lastRB2 = false;

    private int launchStage = -1;

    private enum SpindleMode {
        OPEN,
        ONE_BALL,
        TWO_BALL,
        LAUNCH_1,
        LAUNCH_2,
        LAUNCH_3
    }

    private SpindleMode spindleMode = SpindleMode.OPEN;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final double INTAKE_DEADBAND = 0.05;
    private static final double INTAKE_NORMAL_POWER = 1.0;
    private static final double INTAKE_PRIMED_POWER = 0.5;
    private boolean launcherPrimed = false;

    private static final double COLOR_CAP_CM = .65;

    private static final float GREEN_MIN = 100f;
    private static final float GREEN_MAX = 180f;

    private static final float PURPLE_MIN = 200f;
    private static final float PURPLE_MAX = 280f;

    private boolean artifactWasDetected = false;
    private int artifactCount = 0;

    private double zoneVelOut = 0.0;
    private double finalVelOut = 0.0;
    private double leftVelOut = 0.0;
    private double rightVelOut = 0.0;

    private double headingErrorDeg = 0.0;
    private double goalDistance = 0.0;

    private static final double GOAL_AIM_KP = 0.0275;
    private static final double MAX_AUTO_TURN = 0.35;

    private MecanumDrive drive;

    // Persistent RR pose

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive.updatePoseEstimate();

            LLResult limelightResult = getLimelightResult();
            double targetArea = getTargetArea(limelightResult);


            calculateGoal();

            handleDrive(limelightResult);

            handleArtifactSensorSpindle();
            handleSpindle();

            handleLauncherPower(targetArea);

            handleIntake();
            updateTelemetry(limelightResult, targetArea);

            idle();
        }
        PoseStorage.currentPose = drive.localizer.getPose();
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        intakeSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorLeft");
        intakeDistanceLeft = hardwareMap.get(DistanceSensor.class, "ColorSensorLeft");

        intakeSensorRight = hardwareMap.get(NormalizedColorSensor.class, "ColorSensorRight");
        intakeDistanceRight = hardwareMap.get(DistanceSensor.class, "ColorSensorRight");

        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        spindle = hardwareMap.get(DcMotor.class, "spindle");
        spindle.setTargetPosition(SPINDLE_OPEN);
        spindle.setPower(SPINDLE_POWER);
        spindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        drive = new MecanumDrive(
                hardwareMap,
                PoseStorage.currentPose
        );
    }

    private LLResult getLimelightResult() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result;
        }

        return null;
    }

    private double getTargetArea(LLResult result) {
        if (result == null) return -1.0;
        return result.getTa();
    }

    private double pickVelocityFromTargetArea(double targetArea) {
        double baseVelocity;

        if (targetArea < 0) {
            baseVelocity = NO_TAG_BASE_VELOCITY;
        } else if (targetArea < A1) {
            baseVelocity = V_FAR;
        } else if (targetArea < A2) {
            baseVelocity = V_MID1;
        } else if (targetArea < A3) {
            baseVelocity = V_MID2;
        } else if (targetArea < A4) {
            baseVelocity = V_MID3;
        } else {
            baseVelocity = V_NEAR;
        }

        return Math.max(0, baseVelocity + launchVelocityTrim);
    }
    private double getAutoAimCorrection(LLResult result) {

        double error = -headingErrorDeg;

        // Deadband
        if (Math.abs(error) < AIM_DEADBAND_DEG) {
            return 0.0;
        }

        // Full speed if error is huge
        if (Math.abs(error) > 100) {

            if (error > 0) {
                return 1.0;
            } else {
                return -1.0;
            }
        }

        // Normal proportional control
        double turnPower = error * GOAL_AIM_KP;

        // Clamp
        if (turnPower > MAX_AUTO_TURN) {
            turnPower = MAX_AUTO_TURN;
        }

        if (turnPower < -MAX_AUTO_TURN) {
            turnPower = -MAX_AUTO_TURN;
        }

        return turnPower;
    }
    private void handleDrive(LLResult limelightResult) {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.left_trigger > AUTO_AIM_TRIGGER_DEADBAND) {
            rx = getAutoAimCorrection(limelightResult);
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        double drivePower = getDrivePowerModifier();

        frontLeft.setPower(fl * drivePower);
        backLeft.setPower(bl * drivePower);
        frontRight.setPower(fr * drivePower);
        backRight.setPower(br * drivePower);
    }

    private double getDrivePowerModifier() {
        if (gamepad1.left_bumper) return 0.25;
        if (gamepad1.right_bumper) return 1;
        return .7;
    }

    private void handleLauncherPower(double targetArea) {
        boolean upPressed = gamepad2.dpad_up && !lastDpadUp;
        boolean downPressed = gamepad2.dpad_down && !lastDpadDown;

        if (upPressed) launchVelocityTrim += VELOCITY_TRIM_STEP;
        if (downPressed) launchVelocityTrim -= VELOCITY_TRIM_STEP;

        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left) {
            launcherDirection = -1;
        } else if (gamepad2.dpad_right) {
            launcherDirection = 1;
        }

        zoneVelOut = calculateLaunchVelocity(goalDistance);
        finalVelOut = zoneVelOut;

        boolean fire = (gamepad2.left_trigger > 0.05) || (gamepad2.right_trigger > 0.05);
        launcherPrimed = fire;

        if (fire) {
            launcherLeft.setVelocity((finalVelOut * launcherDirection) * -1);
            launcherRight.setVelocity((finalVelOut * launcherDirection) * -1);
        } else {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }

        leftVelOut = launcherLeft.getVelocity();
        rightVelOut = launcherRight.getVelocity();
    }

    private double calculateLaunchVelocity(double distance) {

        double velocity = 566.1367 * Math.pow(distance, 0.3105518);


        velocity += launchVelocityTrim;

        return Math.max(0, velocity);
    }
    private void setSpindleTarget(int targetTicks, SpindleMode mode) {
        spindle.setTargetPosition(targetTicks);
        spindle.setPower(SPINDLE_POWER);
        spindleMode = mode;
    }

    private void handleArtifactSensorSpindle() {

        double leftDistance = intakeDistanceLeft.getDistance(DistanceUnit.CM);
        double rightDistance = intakeDistanceRight.getDistance(DistanceUnit.CM);

        boolean leftCloseEnough =
                !Double.isNaN(leftDistance) &&
                        leftDistance <= COLOR_CAP_CM;

        boolean rightCloseEnough =
                !Double.isNaN(rightDistance) &&
                        rightDistance <= COLOR_CAP_CM;

        String leftResult = "TOO FAR";
        String rightResult = "TOO FAR";

        if (leftCloseEnough) {
            leftResult = classifyColor(
                    intakeSensorLeft,
                    intakeDistanceLeft,
                    leftHsv
            );
        }

        if (rightCloseEnough) {
            rightResult = classifyColor(
                    intakeSensorRight,
                    intakeDistanceRight,
                    rightHsv
            );
        }

        boolean detected =
                leftResult.equals("GREEN") ||
                        leftResult.equals("PURPLE") ||
                        rightResult.equals("GREEN") ||
                        rightResult.equals("PURPLE");

        if (detected && !artifactWasDetected) {

            artifactCount++;

            if (artifactCount == 1) {

                setSpindleTarget(
                        SPINDLE_1BALL,
                        SpindleMode.ONE_BALL
                );

                launchStage = -1;

            } else if (artifactCount == 2) {

                setSpindleTarget(
                        SPINDLE_2BALL,
                        SpindleMode.TWO_BALL
                );

                launchStage = -1;
            }
        }

        artifactWasDetected = detected;

        if (gamepad2.b) {
            artifactCount = 0;
        }
    }

    private String classifyColor(
            NormalizedColorSensor color,
            DistanceSensor distance,
            float[] hsvOut
    ) {
        double d = distance.getDistance(DistanceUnit.CM);

        if (Double.isNaN(d) || d > COLOR_CAP_CM) {
            return "TOO FAR";
        }

        NormalizedRGBA c = color.getNormalizedColors();

        Color.RGBToHSV(
                (int)(c.red * 255),
                (int)(c.green * 255),
                (int)(c.blue * 255),
                hsvOut
        );

        float h = hsvOut[0];

        if (h >= GREEN_MIN && h <= GREEN_MAX) {
            return "GREEN";
        }

        if (h >= PURPLE_MIN && h <= PURPLE_MAX) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }

    private void handleIntake() {
        double stick = -gamepad2.left_stick_y * .6;

        if (launcherPrimed) {
            if (Math.abs(stick) > INTAKE_DEADBAND) {
                intake.setPower(stick * INTAKE_PRIMED_POWER);
            } else {
                intake.setPower(INTAKE_PRIMED_POWER);
            }

            return;
        }

        if (Math.abs(stick) < INTAKE_DEADBAND) {
            intake.setPower(0);
        } else {
            intake.setPower(stick * INTAKE_NORMAL_POWER);
        }
    }

    private void handleSpindle() {
        boolean aPressed  = gamepad2.a && !lastA2;
        boolean bPressed  = gamepad2.b && !lastB2;
        boolean xPressed  = gamepad2.x && !lastX2;
        boolean rbPressed = gamepad2.right_bumper && !lastRB2;

        if (bPressed) {
            setSpindleTarget(SPINDLE_OPEN, SpindleMode.OPEN);
            launchStage = -1;
        } else if (aPressed) {
            setSpindleTarget(SPINDLE_1BALL, SpindleMode.ONE_BALL);
            launchStage = -1;
        } else if (xPressed) {
            setSpindleTarget(SPINDLE_2BALL, SpindleMode.TWO_BALL);
            launchStage = -1;
        } else if (rbPressed) {
            launchStage = (launchStage + 1) % 3;

            if (launchStage == 0) {
                setSpindleTarget(SPINDLE_LAUNCH_1, SpindleMode.LAUNCH_1);
            } else if (launchStage == 1) {
                setSpindleTarget(SPINDLE_LAUNCH_2, SpindleMode.LAUNCH_2);
            } else {
                setSpindleTarget(SPINDLE_LAUNCH_3, SpindleMode.LAUNCH_3);
            }
        }

        lastA2 = gamepad2.a;
        lastB2 = gamepad2.b;
        lastX2 = gamepad2.x;
        lastRB2 = gamepad2.right_bumper;
    }

    private void calculateGoal() {

        Pose2d pose = drive.localizer.getPose();

        // Current robot position
        double robotX = pose.position.x;
        double robotY = pose.position.y;

        // First calculate distance using FAR target
        // so we can decide which aim point to use
        double farGoalX = 70;
        double goalY = -65;

        double farDx = farGoalX - robotX;
        double farDy = goalY - robotY;

        double rawDistanceToFarGoal =
                Math.sqrt((farDx * farDx) + (farDy * farDy));

        // Choose aim point
        double goalX;

        if (rawDistanceToFarGoal < 120) {
            goalX = 65;
        } else {
            goalX = 70;
        }

        // Final delta to chosen target
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        // Final shooting distance
        goalDistance =
                Math.sqrt((dx * dx) + (dy * dy));

        // Angle robot SHOULD face toward target
        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad);

        // Current robot heading
        double robotHeadingDeg =
                Math.toDegrees(pose.heading.toDouble());

        // Difference between current heading and desired heading
        headingErrorDeg =
                targetAngleDeg - robotHeadingDeg;

        // Normalize to [-180, 180]
        while (headingErrorDeg > 180) {
            headingErrorDeg -= 360;
        }

        while (headingErrorDeg < -180) {
            headingErrorDeg += 360;
        }

        telemetry.addLine("==== GOAL TARGETING ====");

        telemetry.addData("Goal X", "%.2f", goalX);
        telemetry.addData("Goal Y", "%.2f", goalY);

        telemetry.addData("Robot X", "%.2f", robotX);
        telemetry.addData("Robot Y", "%.2f", robotY);

        telemetry.addData("Delta X", "%.2f", dx);
        telemetry.addData("Delta Y", "%.2f", dy);

        telemetry.addData("Goal Distance", "%.2f", goalDistance);

        telemetry.addData("Target Angle Deg", "%.2f", targetAngleDeg);

        telemetry.addData("Robot Heading Deg", "%.2f", robotHeadingDeg);

        telemetry.addData("Heading Error Deg", "%.2f", headingErrorDeg);

        telemetry.addData(
                "Aim Side",
                rawDistanceToFarGoal < 120 ? "CLOSE (65)" : "FAR (70)"
        );
    }

    private void updateTelemetry(LLResult limelightResult, double targetArea) {
        telemetry.addData("Goal Distance", "%.2f", goalDistance);
        telemetry.addData("Heading Error Deg", "%.2f", headingErrorDeg);
        telemetry.addData("Goal Turn Power", "%.3f", getAutoAimCorrection(limelightResult));

        telemetry.addLine("==== ROADRUNNER ====");

        if (drive != null) {
            Pose2d rrPose = drive.localizer.getPose();

            telemetry.addData("RR X", "%.2f", rrPose.position.x);
            telemetry.addData("RR Y", "%.2f", rrPose.position.y);
            telemetry.addData(
                    "RR Heading Deg",
                    "%.2f",
                    Math.toDegrees(rrPose.heading.toDouble())
            );
        } else {
            telemetry.addData("RR X", "N/A");
            telemetry.addData("RR Y", "N/A");
            telemetry.addData("RR Heading Deg", "N/A");
        }

        telemetry.addLine("==== LIMELIGHT ====");

        boolean targetVisible = limelightResult != null;
        telemetry.addData("Limelight Target", targetVisible ? "FOUND" : "NOT FOUND");

        if (targetVisible) {
            telemetry.addData("tx", "%.2f", limelightResult.getTx());
            telemetry.addData("ty", "%.2f", limelightResult.getTy());
            telemetry.addData("Target Area / Tag Size", "%.3f", targetArea);
            telemetry.addData("Auto Aim Correction", "%.3f", getAutoAimCorrection(limelightResult));

            if (targetArea >= 0 && targetArea < A1) {
                telemetry.addData("Aim Zone", "FAR - shifted right");
                telemetry.addData("Aim Target TX", "%.2f", FAR_AIM_TARGET_TX);
            } else {
                telemetry.addData("Aim Zone", "NORMAL");
                telemetry.addData("Aim Target TX", "%.2f", AIM_TARGET_TX);
            }
        } else {
            telemetry.addData("tx", "N/A");
            telemetry.addData("ty", "N/A");
            telemetry.addData("Target Area / Tag Size", "N/A");
            telemetry.addData("Auto Aim Correction", "N/A");
            telemetry.addData("Aim Zone", "N/A");
            telemetry.addData("Aim Target TX", "N/A");
        }

        telemetry.addLine("==== LAUNCHER ====");

        telemetry.addData("ZoneVel Base + Trim(tps)", "%.0f", zoneVelOut);
        telemetry.addData("Launch Velocity Trim", "%.0f", launchVelocityTrim);
        telemetry.addData("FinalVel(tps)", "%.0f", finalVelOut);
        telemetry.addData("LeftVel(tps)", "%.0f", leftVelOut);
        telemetry.addData("RightVel(tps)", "%.0f", rightVelOut);

        telemetry.addLine("==== SPINDLE ====");

        telemetry.addData("Spindle Mode", spindleMode);
        telemetry.addData("Spindle Current (ticks)", "%d", spindle.getCurrentPosition());
        telemetry.addData("Spindle Target  (ticks)", "%d", spindle.getTargetPosition());

        telemetry.addLine("==== INTAKE / COLOR ====");

        telemetry.addData("Artifact Count", artifactCount);
        telemetry.addData("Artifact Detected", artifactWasDetected);
        telemetry.addData("Left Distance cm", "%.1f", intakeDistanceLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance cm", "%.1f", intakeDistanceRight.getDistance(DistanceUnit.CM));

        telemetry.addLine("==== DRIVE ====");

        telemetry.addData("Drive Power", getDrivePowerModifier());

        telemetry.update();
    }
}