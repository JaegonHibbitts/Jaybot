# GPIO setup
# sonar triggering
# echo timing
# distance calculation
# sensor sequencing
# invalid measurement rejection
# median or rolling filtering
# front obstacle classification
# left/right obstacle comparison
# floor/drop-off detection
# stale sensor detection

class SonarSystem:
    def initialize(self):
        ...

    def update(self) -> SonarState:
        ...

    def read_sensor(self, name: str) -> float:
        ...

    def filter_distance(self, name: str, measurement: float) -> float:
        ...

    def classify_front_zone(self, front_cm: float) -> str:
        ...

    def determine_obstacle_side(
        self,
        left_cm: float,
        right_cm: float
    ) -> str:
        ...

    def detect_dropoff(
        self,
        floor_left_cm: float,
        floor_right_cm: float
    ) -> bool:
        ...