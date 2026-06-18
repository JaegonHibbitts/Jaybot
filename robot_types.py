@dataclass
class SonarState:
    front_cm: float
    left_cm: float
    right_cm: float
    floor_left_cm: float
    floor_right_cm: float
    front_zone: str
    dropoff_detected: bool
    obstacle_side: str
    valid: bool


@dataclass
class CameraState:
    steering_bias_deg: float
    confidence: float
    valid: bool


@dataclass
class PerceptionState:
    sonar: SonarState
    camera: CameraState
    emergency_stop: bool


@dataclass
class ArduinoTelemetry:
    left_rpm: float
    right_rpm: float
    left_ticks: int
    right_ticks: int
    motion_status: str
    fault_flags: str


@dataclass
class MotionCommand:
    mode: str
    left_rpm: float
    right_rpm: float
    left_steering_deg: float
    right_steering_deg: float
    distance_cm: float = 0.0