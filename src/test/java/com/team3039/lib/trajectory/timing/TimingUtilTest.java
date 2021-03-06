package com.team3039.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import frc.team3039.utility.lib.geometry.ITranslation2d;
import frc.team3039.utility.lib.geometry.Translation2d;
import frc.team3039.utility.lib.geometry.State;
import frc.team3039.utility.lib.trajectory.DistanceView;
import frc.team3039.utility.lib.trajectory.Trajectory;
import frc.team3039.utility.lib.trajectory.timing.TimedState;
import frc.team3039.utility.lib.trajectory.timing.TimingConstraint;
import frc.team3039.utility.lib.trajectory.timing.TimingConstraint.MinMaxAcceleration;
import frc.team3039.utility.lib.trajectory.timing.TimingUtil;
import frc.team3039.utility.lib.trajectory.timing.VelocityLimitRegionConstraint;
import frc.team3039.utility.Util;

public class TimingUtilTest {

    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0), new Translation2d(36.0, 12.0), new Translation2d(60.0, 12.0));

    public <S extends State<S>> Trajectory<TimedState<S>> buildAndCheckTrajectory(final DistanceView<S> dist_view,
            double step_size, List<TimingConstraint<S>> constraints, double start_vel, double end_vel, double max_vel,
            double max_acc) {
        Trajectory<TimedState<S>> timed_traj = TimingUtil.timeParameterizeTrajectory(false, dist_view, step_size,
                constraints, start_vel, end_vel, max_vel, max_acc);
        checkTrajectory(timed_traj, constraints, start_vel, end_vel, max_vel, max_acc);
        return timed_traj;
    }

    public <S extends State<S>> void checkTrajectory(final Trajectory<TimedState<S>> traj,
            List<TimingConstraint<S>> constraints, double start_vel, double end_vel, double max_vel, double max_acc) {
        assertFalse(traj.isEmpty());
        assertEquals(traj.getState(0).velocity(), start_vel, kTestEpsilon);
        assertEquals(traj.getState(traj.length() - 1).velocity(), end_vel, kTestEpsilon);

        // Go state by state, verifying all constraints are satisfied and integration is
        // correct.
        for (int i = 0; i < traj.length(); ++i) {
            final TimedState<S> state = traj.getState(i);
            for (final TimingConstraint<S> constraint : constraints) {
                assertTrue(state.velocity() - kTestEpsilon <= constraint.getMaxVelocity(state.state()));
                final MinMaxAcceleration accel_limits = constraint.getMinMaxAcceleration(state.state(),
                        state.velocity());
                assertTrue(state.acceleration() - kTestEpsilon <= accel_limits.max_acceleration());
                assertTrue(state.acceleration() + kTestEpsilon >= accel_limits.min_acceleration());
            }
            if (i > 0) {
                final TimedState<S> prev_state = traj.getState(i - 1);
                assertEquals(state.velocity(),
                        prev_state.velocity() + (state.t() - prev_state.t()) * prev_state.acceleration(), kTestEpsilon);
            }
        }
    }

    @Test
    public void testNoConstraints() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        DistanceView<Translation2d> dist_view = new DistanceView<>(traj);

        // Triangle profile.
        Trajectory<TimedState<Translation2d>> timed_traj = buildAndCheckTrajectory(dist_view, 1.0,
                new ArrayList<TimingConstraint<Translation2d>>(), 0.0, 0.0, 20.0, 5.0);
        System.out.println(timed_traj.toCSV());

        // Trapezoidal profile.
        timed_traj = buildAndCheckTrajectory(dist_view, 1.0, new ArrayList<TimingConstraint<Translation2d>>(), 0.0, 0.0,
                10.0, 5.0);
        System.out.println(timed_traj.toCSV());

        // Trapezoidal profile with start and end velocities.
        timed_traj = buildAndCheckTrajectory(dist_view, 1.0, new ArrayList<TimingConstraint<Translation2d>>(), 5.0, 2.0,
                10.0, 5.0);
        System.out.println(timed_traj.toCSV());
    }

    @Test
    public void testConditionalVelocityConstraint() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        DistanceView<Translation2d> dist_view = new DistanceView<>(traj);

        class ConditionalTimingConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
            @Override
            public double getMaxVelocity(S state) {
                if (state.getTranslation().x() >= 24.0) {
                    return 5.0;
                } else {
                    return Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state, double velocity) {
                return new TimingConstraint.MinMaxAcceleration(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
        }

        // Trapezoidal profile.
        Trajectory<TimedState<Translation2d>> timed_traj = buildAndCheckTrajectory(dist_view, 1.0,
                Arrays.asList(new ConditionalTimingConstraint<>()), 0.0, 0.0, 10.0, 5.0);
        System.out.println(timed_traj.toCSV());
    }

    @Test
    public void testConditionalAccelerationConstraint() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        DistanceView<Translation2d> dist_view = new DistanceView<>(traj);

        class ConditionalTimingConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
            @Override
            public double getMaxVelocity(S state) {
                return Double.POSITIVE_INFINITY;
            }

            @Override
            public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state, double velocity) {
                return new TimingConstraint.MinMaxAcceleration(-10.0, 10.0 / velocity);
            }
        }

        // Trapezoidal profile.
        Trajectory<TimedState<Translation2d>> timed_traj = buildAndCheckTrajectory(dist_view, 1.0,
                Arrays.asList(new ConditionalTimingConstraint<>()), 0.0, 0.0, 10.0, 5.0);
        System.out.println(timed_traj.toCSV());
    }

    @Test
    public void testVelocityLimitRegionConstraint() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        DistanceView<Translation2d> dist_view = new DistanceView<>(traj);

        VelocityLimitRegionConstraint<Translation2d> constraint = new VelocityLimitRegionConstraint<>(
                new Translation2d(6.0, -6.0), new Translation2d(18.0, 6.0), 3.0);

        // Trapezoidal profile.
        Trajectory<TimedState<Translation2d>> timed_traj = buildAndCheckTrajectory(dist_view, 1.0,
                Arrays.asList(constraint), 0.0, 0.0, 10.0, 5.0);
        System.out.println(timed_traj.toCSV());
    }

}
