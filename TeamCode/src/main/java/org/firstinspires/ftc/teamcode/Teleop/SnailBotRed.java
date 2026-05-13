package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Snail Bot Red", group = "TeleOp")
public class SnailBotRed extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor spindle;
    private DcMotorEx launcherLeft, launcherRight;
    private HuskyLens camera;

    // REV Color Sensor V3
    private NormalizedColorSensor intakeSensorLeft, intakeSensorRight;
    private DistanceSensor intakeDistanceLeft, intakeDistanceRight;

    private final float[] leftHsv = new float[3];
    private final float[] rightHsv = new float[3];

    private int launcherDirection = 1;

    private static final double H1 = 25;
    private static final double H2 = 30;
    private static final double H3 = 35;
    private static final double H4 = 40;

    private static final double V_FAR  = 910;
    private static final double V_MID1 = 900;
    private static final double V_MID2 = 790;
    private static final double V_MID3 = 775;
    private static final double V_NEAR = 800;

    private double noTagVelocity = 750;
    private static final double NO_TAG_STEP = 50;

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

    private enum SpindleMode { OPEN, ONE_BALL, TWO_BALL, LAUNCH_1, LAUNCH_2, LAUNCH_3 }
    private SpindleMode spindleMode = SpindleMode.OPEN;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final double INTAKE_DEADBAND = 0.05;
    private static final double INTAKE_NORMAL_POWER = 1.0;
    private static final double INTAKE_PRIMED_POWER = 0.35;
    private boolean launcherPrimed = false;

    private static final double LAUNCH3_VEL_MULT = 0.85;

    final int TargetTagID = 5;

    // Color detection settings copied from FullBlue
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

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            handleDrive();

            handleArtifactSensorSpindle();
            handleSpindle();

            double tagHeight = readTagHeight();
            handleLauncherPower(tagHeight);

            handleIntake();
            updateTelemetry(tagHeight);

            idle();
        }
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

        camera = hardwareMap.get(HuskyLens.class, "huskylens");
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    private double pickVelocityFromTagHeight(double tagHeight) {
        if (tagHeight < 0) return noTagVelocity;

        if (tagHeight < H1) return V_FAR;
        if (tagHeight < H2) return V_MID1;
        if (tagHeight < H3) return V_MID2;
        if (tagHeight < H4) return V_MID3;
        return V_NEAR;
    }

    private double readTagHeight() {
        double height = -1.0;

        HuskyLens.Block[] blocks = camera.blocks();
        for (HuskyLens.Block b : blocks) {
            if (b.id == TargetTagID) {
                height = b.height;
                break;
            }
        }

        return height;
    }

    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

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

    private void handleLauncherPower(double tagHeight) {
        boolean upPressed = gamepad2.dpad_up && !lastDpadUp;
        boolean downPressed = gamepad2.dpad_down && !lastDpadDown;

        if (upPressed) noTagVelocity += NO_TAG_STEP;
        if (downPressed) noTagVelocity -= NO_TAG_STEP;

        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left) {
            launcherDirection = -1;
        } else if (gamepad2.dpad_right) {
            launcherDirection = 1;
        }

        zoneVelOut = pickVelocityFromTagHeight(tagHeight);
        finalVelOut = zoneVelOut;

        if (launchStage == 2) {
            finalVelOut *= LAUNCH3_VEL_MULT;
        }

        boolean fire = (gamepad2.left_trigger > 0.05) || (gamepad2.right_trigger > 0.05);
        launcherPrimed = fire;

        if (fire) {
            launcherLeft.setVelocity(finalVelOut * launcherDirection);
            launcherRight.setVelocity(finalVelOut * launcherDirection);
        } else {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }

        leftVelOut = launcherLeft.getVelocity();
        rightVelOut = launcherRight.getVelocity();
    }

    private void setSpindleTarget(int targetTicks, SpindleMode mode) {
        spindle.setTargetPosition(targetTicks);
        spindle.setPower(SPINDLE_POWER);
        spindleMode = mode;
    }

    private void handleArtifactSensorSpindle() {

        double leftDistance =
                intakeDistanceLeft.getDistance(DistanceUnit.CM);

        double rightDistance =
                intakeDistanceRight.getDistance(DistanceUnit.CM);

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

        // Rising edge only
        if (detected && !artifactWasDetected) {

            artifactCount++;

            if (artifactCount == 1) {

                // Same as pressing A
                setSpindleTarget(
                        SPINDLE_1BALL,
                        SpindleMode.ONE_BALL
                );

                launchStage = -1;

            } else if (artifactCount == 2) {

                // Same as pressing X
                setSpindleTarget(
                        SPINDLE_2BALL,
                        SpindleMode.TWO_BALL
                );

                launchStage = -1;
            }
        }

        artifactWasDetected = detected;

        // Reset count when B is pressed
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
        double stick = -gamepad2.left_stick_y*.6;

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

    private void updateTelemetry(double tagHeight) {
        String leftColorResult = classifyColor(intakeSensorLeft, intakeDistanceLeft, leftHsv);
        String rightColorResult = classifyColor(intakeSensorRight, intakeDistanceRight, rightHsv);

        telemetry.addData("Drive Power", getDrivePowerModifier());
        telemetry.addData("Tag Height", tagHeight);

        boolean tagVisible = tagHeight >= 0;
        telemetry.addData("Tag", tagVisible ? "FOUND" : "NOT FOUND");

        telemetry.addData("ZoneVel(tps)", "%.0f", zoneVelOut);
        telemetry.addData("LeftVel(tps)", "%.0f", leftVelOut);
        telemetry.addData("RightVel(tps)", "%.0f", rightVelOut);

        telemetry.addData("Spindle Mode", spindleMode);
        telemetry.addData("Spindle Current (ticks)", "%d", spindle.getCurrentPosition());
        telemetry.addData("Spindle Target  (ticks)", "%d", spindle.getTargetPosition());

        telemetry.addData("Artifact Count", artifactCount);
        telemetry.addData("Artifact Detected", artifactWasDetected);

        telemetry.addData("Left Color", leftColorResult);
        telemetry.addData("Right Color", rightColorResult);
        telemetry.addData("Left Hue", "%.1f", leftHsv[0]);
        telemetry.addData("Right Hue", "%.1f", rightHsv[0]);
        telemetry.addData("Left Distance cm", "%.1f", intakeDistanceLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance cm", "%.1f", intakeDistanceRight.getDistance(DistanceUnit.CM));

        telemetry.update();
    }
}