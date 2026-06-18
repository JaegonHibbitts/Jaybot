# receive SonarState
# receive CameraState
# apply validity rules
# determine whether a hard emergency exists
# produce one PerceptionState

class PerceptionFusion:
    def combine(
        self,
        sonar: SonarState,
        camera: CameraState
    ) -> PerceptionState:

        emergency = (
            sonar.dropoff_detected
            or not sonar.valid
        )

        return PerceptionState(
            sonar=sonar,
            camera=camera,
            emergency_stop=emergency
        )