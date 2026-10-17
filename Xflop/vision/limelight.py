from dataclasses import dataclass
from ntcore import NetworkTableInstance
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d


@dataclass
class PoseEstimate:
    pose: Pose2d
    timestamp_seconds: float
    tag_count: int


class Limelight:
    def __init__(self, name: str = "limelight"):
        table = NetworkTableInstance.getDefault().getTable(name)
        self._tx = table.getDoubleTopic("tx").subscribe(0.0)
        self._ta = table.getDoubleTopic("ta").subscribe(0.0)
        self._id_filter_pub = table.getDoubleArrayTopic("fiducial_id_filters_set").publish()
        self._tv = table.getDoubleTopic("tv").subscribe(0.0)
        self._tid = table.getDoubleTopic("tid").subscribe(-1.0)

        self._botpose = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([])
        self._orientation_pub = table.getDoubleArrayTopic("robot_orientation_set").publish()

    def has_target(self) -> bool:
        return self._tv.get() > 0.5

    def set_robot_orientation(self, yaw_degrees: float) -> None:
        self._orientation_pub.set([yaw_degrees, 0.0, 0.0, 0.0, 0.0, 0.0])

    def get_pose_estimate(self):
        data = self._botpose.get()
        # [x, y, z, roll, pitch, yaw, latencia, num_tags, span, dist_prom, area_prom]
        if len(data) < 8 or int(data[7]) < 1:
            return None  # sin Limelight, sin conexión, o sin tags visibles
        pose = Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]))
        timestamp = Timer.getFPGATimestamp() - (data[6] / 1000.0)
        return PoseEstimate(pose, timestamp, int(data[7]))

    def get_tx(self) -> float:
        return self._tx.get()

    def get_ta(self) -> float:
        return self._ta.get()

    def set_fiducial_id_filter(self, tag_ids) -> None:
        self._id_filter_pub.set([float(i) for i in tag_ids])

    def get_tid(self) -> int:
        return int(self._tid.get())