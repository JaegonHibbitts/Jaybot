# This file should manage states such as:

# STARTUP
# CRUISE
# SOFT_AVOID
# BRAKE
# REVERSE_RECOVERY
# ESCAPE_TURN
# STOPPED
# FAULT

# It should decide:

# What state should JayBot be in?
# When should a state begin?
# When is a state complete?
# Which direction should an escape turn use?
# Should a camera bias be ignored?
# Should a drop-off cause a latched stop?

class BehaviorManager:
    def __init__(self):
        self.state = "STARTUP"
        self.escape_direction = "LEFT"
        self.state_start_time = 0.0

    def update(
        self,
        perception: PerceptionState,
        telemetry: ArduinoTelemetry
    ) -> str:
        ...

    def check_safety(self, perception: PerceptionState) -> bool:
        ...

    def select_escape_direction(
        self,
        perception: PerceptionState
    ) -> str:
        ...

    def transition_to(self, new_state: str):
        ...