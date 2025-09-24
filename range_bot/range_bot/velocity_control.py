from threading import RLock
from collections import deque
from statistics import stdev

from range_bot.state_machine import StateMachine, BotState
from more_msgs.msg import EstopStamped, RangefinderStamped, TelemetryStamped


import rclpy
from rclpy.node import Node


class RangeBot(Node):
    def __init__(
        self,
        slow_distance: float = 800,
        stop_distance: float = 400,
        full_velocity: float = 1000.0,
        slow_velocity: float = 300.0,
        smooth_accel_limit: float = 200.0,
        estop_accel_limit: float = 2000.0,
        update_rate: float = 10.0,
    ):
        """
        A simple range bot that uses a state machine to control its velocity based on distance to
        nearest obstacle.
        Args:
            slow_distance (float): Distance to nearest obstacle in mm at which to transition to
                SLOW state. Defaults to 800mm.
            stop_distance (float): Distance to nearest obstacle in mm at which to transition to
                STOP state. Defaults to 400mm.
            full_velocity (float): Velocity in mm/s for FULL_SPEED state. Defaults to 1000mm/s.
            slow_velocity (float): Velocity in mm/s for SLOW state. Defaults to 300mm/s.
            smooth_accel_limit (float): Maximum acceleration in mm/s^2 for normal operation.
                Defaults to 200mm/s^2.
            estop_accel_limit (float): Maximum acceleration in mm/s^2 for emergency stop operation.
                Defaults to 2000mm/s^2.
            update_rate (float): Rate in Hz at which to update the state machine and velocity.
                Defaults to 10Hz.
        """
        super().__init__("range_bot")
        self.state_machine = StateMachine(
            self.get_logger(), slow_distance=slow_distance, stop_distance=stop_distance
        )
        self.current_velocity = 0.0  # Current velocity in mm/s
        self.current_distance = 0.0  # Current distance to nearest obstacle in mm
        self.set_velocity_setpoint = 0.0  # Desired velocity in mm/s
        self._lock = RLock()  # To make bot thread-safe
        self.full_velocity = full_velocity  # Max velocity in mm/s
        self.slow_velocity = slow_velocity  # Slow velocity in mm/s
        self.smooth_accel_limit = smooth_accel_limit  # Max accel in mm/s^2 in normal operation
        self.estop_accel_limit = estop_accel_limit  # Max accel in mm/s^2 during estop
        self._queue_len = 10  # Outlier rejection queue length
        self._rejection_count = 0  # Number of outliers rejected in sequence
        self._rf_queue = deque(maxlen=self._queue_len)  # rangefinder data for outlier rejection
        self.update_rate = update_rate  # Rate in Hz to update the state machine and velocity
        self._last_stamp = self.get_clock().now()  # Last time the update function ran.
        # Implement a timer to call the update function at the specified rate
        self.timer = self.create_timer(1.0 / self.update_rate, self.update)
        self.estop_subscriber = self.create_subscription(
            EstopStamped, "estop", self.estop_callback, 10
        )
        self._rangefinder_subscriber = self.create_subscription(
            RangefinderStamped, "rangefinder", self.rangefinder_callback, 10
        )
        self._telemetry_publisher = self.create_publisher(TelemetryStamped, "telemetry", 10)
        self.get_logger().info("range_bot initialized.")

    @property
    def state(self) -> BotState:
        """Get the current state of the bot."""
        return self.state_machine.state

    def rangefinder_callback(self, data: RangefinderStamped):
        """
        Callback function for rangefinder data. This function should be called whenever new
        rangefinder data is available. Only sets rangefinder data. Current distance is set based on
        new readings with outlier rejection. 3-sigma rule is used for outlier rejection.
        Args:
            data (RangefinderStamped): Distance to nearest obstacle in mm.
        """
        # If the rejection count is more than half the queue length, flush the queue and start over
        if self._rejection_count > self._queue_len / 2:
            self._rf_queue.clear()

        distance = data.distance
        # Accept distance if queue is empty
        if not self._rf_queue:
            self.current_distance = distance

        # Once we have multiple samples, we can compute standard deviation to check for outliers.
        # No readings are accepted until this is possible.
        if len(self._rf_queue) > 2:
            std_dev = stdev(self._rf_queue)
            # 3-sigma rule for outlier rejection. If the new reading is more than 3 standard
            # deviations from the current distance, it is considered an outlier. Outliers are not
            # accepted as a reading but remain in the queue for future calculations.
            if abs(distance - self.current_distance) > 3 * std_dev:
                self.get_logger().warn(f"Outlier detected: {distance} mm, ignoring.")
                self._rejection_count += 1
            else:
                # Update current distance to last value in the queue
                self.current_distance = distance
                self.get_logger().info(f"Updated distance: {self.current_distance} mm")
                self._rejection_count = 0

        # Update queue of rangefinder readings, new findings are always added to the end
        self._rf_queue.append(distance)

    def estop_callback(self, data: EstopStamped):
        """
        Callback function for emergency stop. This function should be called whenever the estop
        state changes. Immediately activates or deactivates the estop in the state machine and
        triggers
        Args:
            data (EstopStamped): True to activate Estop, False to deactivate Estop.
        """
        with self._lock:
            estopped = data.estop_active
            if estopped:
                self.state_machine.activate_estop()
            else:
                # When deactivating estop, provide a distance to determine the next state
                self.state_machine.deactivate_estop(self.current_distance)

    def get_velocity_setpoint(self) -> float:
        """
        Returns a target velocity based on the current state of the state machine.
        Returns:
            float: The target velocity in mm/s.
        """
        with self._lock:
            # If the state is STOP, ESTOPPED, the velocity should be zero.
            if self.state == BotState.ESTOPPED or self.state == BotState.STOP:
                return 0.0
            elif self.state == BotState.FULL_SPEED:
                return self.full_velocity  # Full speed ahead
            elif self.state == BotState.SLOW:
                return self.slow_velocity  # Slow speed
            # In all other cases, return zero velocity
            self.get_logger().warn(f"Unhandled state {self.state}, setting velocity to 0.0")
        return 0.0  # Stop

    def update(self):
        """
        Update the state machine and velocity based on the elapsed time.
        """
        # Placed under lock so update can be called by ESTOP callback if needed
        with self._lock:
            current_distance = self.current_distance
            # Update the state machine based on the current distance
            self.state_machine.set_state_by_distance(current_distance)

            # Get the desired velocity setpoint based on the current state
            velocity_setpoint = self.get_velocity_setpoint()

            # Determine the appropriate acceleration limit
            if self.state == BotState.ESTOPPED:
                accel_limit = self.estop_accel_limit
            else:
                accel_limit = self.smooth_accel_limit

            # Calculate the maximum change in velocity allowed this update cycle
            current_stamp = self.get_clock().now()
            max_delta_v = accel_limit * (current_stamp - self._last_stamp).nanoseconds / 1e9

            # Calculate the difference between the desired and current velocity
            delta_v = velocity_setpoint - self.current_velocity

            # Limit the change in velocity to the maximum allowed
            if abs(delta_v) > max_delta_v:
                delta_v = max_delta_v if delta_v > 0 else -max_delta_v

            # Normally this would be updated by sensor or ekf feedback, but for this example we
            # assume the velocity changes as commanded.
            self.current_velocity += delta_v

            # Update the last stamp to the current time
            self._last_stamp = current_stamp

            self.get_logger().debug(
                f"State: {self.state}, Distance: {current_distance} mm, "
                f"Setpoint: {velocity_setpoint} mm/s, "
                f"Current Velocity: {self.current_velocity} mm/s"
                f"Stamp: {self._last_stamp.nanoseconds / 1e9}s, ",
            )

            # Construct and publish telemetry update
            telemetry = TelemetryStamped()
            telemetry.header.stamp = current_stamp.to_msg()
            telemetry.header.frame_id = "range_bot"
            telemetry.velocity = self.current_velocity
            telemetry.velocity_setpoint = velocity_setpoint
            telemetry.rangefinder_distance = current_distance
            telemetry.state.data = self.state.name
            self._telemetry_publisher.publish(telemetry)


def main(args=None):
    rclpy.init(args=args)
    node = RangeBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl-C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
