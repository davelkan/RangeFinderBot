from threading import RLock
import enum


class BotState(enum.Enum):
    UNDEFINED = -1
    ESTOPPED = 0
    STOP = 1
    SLOW = 2
    FULL_SPEED = 3

    def __sub__(self, other):
        return self.value - other.value


class StateMachine:
    def __init__(
        self,
        logger=None,
        slow_distance: float = 800,
        stop_distance: float = 400,
        mode_hysteresis: float = 10,
    ):
        """
        A simple state machine for a range bot. The state machine transitions between states based
        on distance to nearest obstacle and will not transition out of ESTOPPED until manually
        reset. The state will begun in the STOP state, and will transition based on distance to
        nearest obstacle or activation of ESTOP.
        Args:
            logger Optional(rclpy.Logger): Logging interface passed by implementing node.
            slow_distance (float): Distance to nearest obstacle in mm at which to transition to
                SLOW state. Defaults to 800mm.
            stop_distance (float): Distance to nearest obstacle in mm at which to transition to
                STOP state. Defaults to 400mm.
            mode_hysteresis (float): A buffer when switching between adjacent modes. Defaults to
                10mm.
        """
        self._logger = logger
        self._state = BotState.STOP
        self._slow_distance = slow_distance
        self._stop_distance = stop_distance
        self._mode_hysteresis = mode_hysteresis
        self._lock = RLock()  # To make state machine thread-safe
        if self._logger is not None:
            self._logger.info(f"Initialized in state: {self._state}")

    @property
    def state(self) -> BotState:
        """Get the current state of the state machine."""
        with self._lock:
            return self._state

    def _transition(self, new_state: BotState):
        """Transition to a new state. This method should only be called internally by the state
        machine.
        Args:
            new_state (BotState): The state to transition to.
        """
        with self._lock:
            if self._state != new_state:
                if self._logger is not None:
                    self._logger.info(f"Transitioning from {self._state} to {new_state}")
                self._state = new_state

    def activate_estop(self):
        """Activate the emergency stop. This will transition the state machine to the ESTOPPED
        state. The state machine will not transition out of ESTOPPED until manually reset.
        """
        with self._lock:
            self._transition(BotState.ESTOPPED)
            # Confirm that the state is now ESTOPPED
            if self._state != BotState.ESTOPPED:
                if self._logger is not None:
                    self._logger.warn("Estop activation failed. State is not ESTOPPED.")

    def deactivate_estop(self, distance: float):
        """Deactivate the emergency stop. The state machine will transition to a
        new state based on distance provided. The state machine will always briefly transition to
        BotState.STOP if previously estopped prior to selecting a new state based on the
        rangefinder.
        Args:
            distance (float): Distance to nearest obstacle in mm.
        """
        with self._lock:
            # Trigger a warning in case estop not active, but continue anyways.
            prior_state_is_estopped = self._state == BotState.ESTOPPED
            if not prior_state_is_estopped:
                if self._logger is not None:
                    self._logger.warn("Estop not active. Setting state based on distance sensor.")
            else:
                if self._logger is not None:
                    self._logger.info("Deactivating estop.")
                self._transition(BotState.STOP)

            # Attempt to transition to a new state based on distance. Switch first to stop state.
            self.set_state_by_distance(distance)
            if self._logger is not None:
                self._logger.info(f"State transitioned to {self._state}")

    def set_state_by_distance(self, distance: float):
        """Select state based on distance to nearest obstacle. If state is currently estopped,
        do not change state. Includes a simple hysteresis to limit rapid switching between adjacent
        states.
        Args:
            distance (float): Distance to nearest obstacle in meters.
        """
        with self._lock:
            # Remain in ESTOPPED until manually reset. Do not change state based on distance.
            if self._state == BotState.ESTOPPED:
                return

            # Default to current state
            desired_state = self._state

            # Not Estopped, transition logic will depend on current state due to hysteresis.
            if self._state == BotState.STOP:
                # STOP has hysteresis only when switching to SLOW
                if distance > self._slow_distance:
                    desired_state = BotState.FULL_SPEED
                elif distance > self._stop_distance + self._mode_hysteresis:
                    desired_state = BotState.SLOW
                else:
                    desired_state = BotState.STOP
            elif self._state == BotState.SLOW:
                # SLOW has a hysteresis on transitions to STOP and to FULL_SPEED
                if distance < self._stop_distance - self._mode_hysteresis:
                    desired_state = BotState.STOP
                elif distance > self._slow_distance + self._mode_hysteresis:
                    desired_state = BotState.FULL_SPEED
                else:
                    desired_state = BotState.SLOW
            elif self._state == BotState.FULL_SPEED:
                # FULL_SPEED has hysteresis only when switching to SLOW
                if distance < self._stop_distance:
                    desired_state = BotState.STOP
                elif distance < self._slow_distance - self._mode_hysteresis:
                    desired_state = BotState.SLOW
                else:
                    desired_state = BotState.FULL_SPEED

            # If the state has changed, _transition to the new state
            if desired_state != self._state:
                self._transition(desired_state)
