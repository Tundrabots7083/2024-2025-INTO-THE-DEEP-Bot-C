package org.firstinspires.ftc.teamcode.ftc7083.feedback.profile;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;

/**
     * PID controller with various feedforward components.
     */
    public final class OLD_PIDFController {


        private final PIDCoefficients pid;
        private final double kV, kA, kStatic;
        private final FeedForward kF;

        private double errorSum;
        private long lastUpdateTs;

        private boolean inputBounded;
        private double minInput, maxInput;

        private boolean outputBounded;
        private double minOutput, maxOutput;

        /**
         * Target position (that is, the controller setpoint).
         */
        public double targetPosition;

        /**
         * Target velocity.
         */
        public double targetVelocity;

        /**
         * Target acceleration.
         */
        public double targetAcceleration;

        /**
         * Error computed in the last call to {@link #update(long, double, Double)}
         */
        public double lastError;

        /**
         * Feedforward parameters {@code kV}, {@code kA}, and {@code kStatic} correspond with a basic
         * kinematic model of DC motors. The general function {@code kF} computes a custom feedforward
         * term for other plants.
         *
         * @param pid traditional PID coefficients
         * @param kV feedforward velocity gain
         * @param kA feedforward acceleration gain
         * @param kStatic additive feedforward constant
         * @param kF custom feedforward that depends on position and velocity
         */
        public OLD_PIDFController(
                PIDCoefficients pid,
                double kV,
                double kA,
                double kStatic,
                FeedForward kF
        ) {
            this.pid = pid;
            this.kV = kV;
            this.kA = kA;
            this.kStatic = kStatic;
            this.kF = kF;
        }

        public OLD_PIDFController(
                PIDCoefficients pid,
                double kV,
                double kA,
                double kStatic
        ) {
            this(pid, kV, kA, kStatic, (g) -> 0);
        }

        public OLD_PIDFController(
                PIDCoefficients pid,
                FeedForward kF
        ) {
            this(pid, 0, 0, 0, kF);
        }

        public OLD_PIDFController(
                PIDCoefficients pid
        ) {
            this(pid, 0, 0, 0);
        }

        /**
         * Sets bound on the input of the controller. When computing the error, the min and max are
         * treated as the same value. (Imagine taking the segment of the real line between min and max
         * and attaching the endpoints.)
         *
         * @param min minimum input
         * @param max maximum input
         */
        public void setInputBounds(double min, double max) {
            if (min < max) {
                inputBounded = true;
                minInput = min;
                maxInput = max;
            }
        }

        /**
         * Sets bounds on the output of the controller.
         *
         * @param min minimum output
         * @param max maximum output
         */
        public void setOutputBounds(double min, double max) {
            if (min < max) {
                outputBounded = true;
                minOutput = min;
                maxOutput = max;
            }
        }

        /**
        * sets the target position
         * @param targetPosition the target position to go to
         */
        public void setTargetPosition(double targetPosition) {
                this.targetPosition = targetPosition;
        }

        /**
         * Sets the target velocity
        * @param targetVelocity velocity to go to
        */
        public void setTargetVelocity(double targetVelocity) {
            this.targetVelocity = targetVelocity;
        }

        /**
         * Sets the target acceleration
         * @param targetAcceleration acceleration to go to
         */
        public void setTargetAcceleration(double targetAcceleration) {
            this.targetAcceleration = targetAcceleration;
        }

        private double getPositionError(double measuredPosition) {
            double error = targetPosition - measuredPosition;
            if (inputBounded) {
                final double inputRange = maxInput - minInput;
                while (Math.abs(error) > inputRange / 2.0) {
                    error -= Math.copySign(inputRange, error);
                }
            }
            return error;
        }

        /**
         * Run a single iteration of the controller.
         *
         * @param timestamp measurement timestamp as given by {@link System#nanoTime()}
         * @param measuredPosition measured position (feedback)
         * @param measuredVelocity measured velocity
         */
        public double update(
                long timestamp,
                double measuredPosition,
                @Nullable Double measuredVelocity
        ) {
            final double error = getPositionError(measuredPosition);

            if (lastUpdateTs == 0) {
                lastError = error;
                lastUpdateTs = timestamp;
                return 0;
            }

            final double dt = timestamp - lastUpdateTs;
            errorSum += 0.5 * (error + lastError) * dt;
            final double errorDeriv = (error - lastError) / dt;

            lastError = error;
            lastUpdateTs = timestamp;

            double velError;
            if (measuredVelocity == null) {
                velError = errorDeriv;
            } else {
                velError = targetVelocity - measuredVelocity;
            }

            double baseOutput = pid.p * error + pid.i * errorSum + pid.d * velError +
                    kV * targetVelocity + kA * targetAcceleration +
                    kF.calculate(targetPosition);

            double output = 0;
            if (Math.abs(baseOutput) > 1e-6) {
                output = baseOutput + Math.copySign(kStatic, baseOutput);
            }

            if (outputBounded) {
                return Math.max(minOutput, Math.min(output, maxOutput));
            }

            return output;
        }

        public double update(
                long timestamp,
                double measuredPosition
        ) {
            return update(timestamp, measuredPosition, null);
        }

        public double update(
                double measuredPosition
        ) {
            return update(System.nanoTime(), measuredPosition, null);
        }

        /**
         * Reset the controller's integral sum.
         */
        public void reset() {
            errorSum = 0;
            lastError = 0;
            lastUpdateTs = 0;
        }
    }
