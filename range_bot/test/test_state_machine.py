from range_bot.state_machine import StateMachine, BotState


def test_state_machine():
    """Test the StateMachine class for correctness."""
    sm = StateMachine(slow_distance=800, stop_distance=400)

    # Initial state should be STOP
    assert sm.state == BotState.STOP

    # Test transition to FULL_SPEED
    sm.set_state_by_distance(distance=1000)
    assert sm.state == BotState.FULL_SPEED

    # Test transition to SLOW
    sm.set_state_by_distance(distance=600)
    assert sm.state == BotState.SLOW

    # Test transition to STOP
    sm.set_state_by_distance(distance=300)
    assert sm.state == BotState.STOP

    # Test transition back to FULL_SPEED
    sm.set_state_by_distance(distance=1000)
    assert sm.state == BotState.FULL_SPEED

    # Activate estop and ensure state is ESTOPPED
    sm.activate_estop()
    assert sm.state == BotState.ESTOPPED

    # Attempt to change state to full speed while ESTOPPED; should remain ESTOPPED
    sm.set_state_by_distance(distance=1000)
    assert sm.state == BotState.ESTOPPED

    # Attempt to change state to slow while ESTOPPED; should remain ESTOPPED
    sm.set_state_by_distance(distance=600)
    assert sm.state == BotState.ESTOPPED

    # Attempt to change state to stop while ESTOPPED; should remain ESTOPPED
    sm.set_state_by_distance(distance=300)
    assert sm.state == BotState.ESTOPPED

    # Deactivate estop and ensure it transitions based on distance
    sm.deactivate_estop(distance=600)
    assert sm.state == BotState.SLOW

    # Deactivate estop again (should log a warning but continue)
    sm.deactivate_estop(distance=1000)
    assert sm.state == BotState.FULL_SPEED

    # Validate hysteresis FULL_SPEED -> SLOW
    sm.set_state_by_distance(distance=791)
    assert sm.state == BotState.FULL_SPEED
    sm.set_state_by_distance(distance=789)
    assert sm.state == BotState.SLOW

    # Validate hysteresis SLOW -> FULL_SPEED
    sm.set_state_by_distance(distance=801)
    assert sm.state == BotState.SLOW
    sm.set_state_by_distance(distance=811)
    assert sm.state == BotState.FULL_SPEED

    # Validate lack of hysteresis FULL_SPEED -> STOP
    sm.set_state_by_distance(distance=399)
    assert sm.state == BotState.STOP

    # Validate hysteresis STOP -> SLOW
    sm.set_state_by_distance(distance=401)
    assert sm.state == BotState.STOP
    sm.set_state_by_distance(distance=411)
    assert sm.state == BotState.SLOW

    # Validate hysteresis SLOW -> STOP
    sm.set_state_by_distance(distance=399)
    assert sm.state == BotState.SLOW
    sm.set_state_by_distance(distance=389)
    assert sm.state == BotState.STOP
