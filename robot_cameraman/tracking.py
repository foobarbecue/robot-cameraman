import logging
import time
from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum, auto, IntEnum
from logging import Logger
from typing import Optional

from typing_extensions import Protocol

from robot_cameraman.box import Box, TwoPointsBox, Point
from robot_cameraman.live_view import ImageSize

logger: Logger = logging.getLogger(__name__)


class Destination:

    def __init__(self, image_size: ImageSize, variance: int = 50) -> None:
        width, height = image_size
        x, y = width / 2, height / 2
        self.center = Point(x, y)
        self.box = Box.from_center_and_size(self.center,
                                            variance * 2,
                                            variance * 2)
        self.variance = variance
        x_padding = 0.3 * width
        y_padding = 0.2 * height
        self.min_size_box = TwoPointsBox(0, 0, 0, 0)
        self.max_size_box = TwoPointsBox(0, 0, 0, 0)
        self.max_size_box.width = width - 2 * x_padding
        self.max_size_box.height = height - 2 * y_padding
        self.min_size_box.width = self.max_size_box.width - 2 * self.variance
        self.min_size_box.height = self.max_size_box.height - 2 * self.variance
        self.update_size_box_center(x, y)

    def update_size_box_center(self, x: float, y: float):
        self.max_size_box.x = x - self.max_size_box.width / 2
        self.max_size_box.y = y - self.max_size_box.height / 2
        self.max_size_box.center.set(x, y)

        self.min_size_box.x = x - self.min_size_box.width / 2
        self.min_size_box.y = y - self.min_size_box.height / 2
        self.min_size_box.center.set(x, y)


class ZoomSpeed(IntEnum):
    ZOOM_OUT_FAST = -200
    ZOOM_OUT_SLOW = -100
    ZOOM_STOPPED = 0
    ZOOM_IN_SLOW = 100
    ZOOM_IN_FAST = 200


@dataclass
class CameraSpeeds:
    pan_speed: float = 0
    tilt_speed: float = 0
    zoom_speed: ZoomSpeed = ZoomSpeed.ZOOM_STOPPED

    def reset(self):
        self.pan_speed = 0
        self.tilt_speed = 0
        self.zoom_speed = ZoomSpeed.ZOOM_STOPPED


class TrackingStrategy(Protocol):
    @abstractmethod
    def update(
            self,
            camera_speeds: CameraSpeeds,
            target: Optional[Box],
            is_target_lost: bool) -> None:
        raise NotImplementedError


class SimpleTrackingStrategy(TrackingStrategy):
    _destination: Destination
    _image_size: ImageSize
    max_allowed_speed: float

    def __init__(
            self,
            destination: Destination,
            image_size: ImageSize,
            max_allowed_speed: float = 1000):
        self._destination = destination
        self._image_size = image_size
        self.max_allowed_speed = max_allowed_speed

    def update(
            self,
            camera_speeds: CameraSpeeds,
            target: Optional[Box],
            is_target_lost: bool) -> None:
        if target is None or is_target_lost:
            return
        tx, ty = target.center
        self._destination.update_size_box_center(tx, ty)
        dx, dy = self._destination.center
        camera_speeds.pan_speed = \
            self._get_speed_by_distance(tx, dx, self._image_size.width)
        camera_speeds.tilt_speed = \
            self._get_speed_by_distance(ty, dy, self._image_size.height)
        self._update_zoom_speed(camera_speeds, target)

    def _update_zoom_speed(self, camera_speeds, target):
        if target.height < self._destination.min_size_box.height:
            camera_speeds.zoom_speed = ZoomSpeed.ZOOM_IN_FAST
        elif target.height > self._destination.max_size_box.height:
            camera_speeds.zoom_speed = ZoomSpeed.ZOOM_OUT_FAST
        else:
            camera_speeds.zoom_speed = ZoomSpeed.ZOOM_STOPPED

    def _get_speed_by_distance(
            self, target: float, destination: float, size: int) -> float:
        distance = target - destination
        abs_distance = abs(distance)
        if abs_distance < self._destination.variance:
            return 0
        else:
            speed = abs_distance / (size / 2) * self.max_allowed_speed
            speed = min(self.max_allowed_speed, speed)
            if distance < 0:
                speed = -speed
            return speed


class TrackingStrategyRotationMode(Enum):
    STOP = auto()
    """Stop rotation, when distance of object to center is smaller than
    the variance."""

    LINEAR = auto()
    "Rotation speed increases linear based on distance of object to center."

    QUADRATIC = auto()
    "Rotation speed increases quadratic based on distance of object to center."

    QUADRATIC_TO_LINEAR = auto()
    """Rotation speed increases quadratic based on distance of object to center,
     when distance of object to center is smaller than the variance.
     Otherwise, rotation speed increases linear based on distance of object
     to center."""


class TrackingStrategyZoomInMode(Enum):
    SLOW = auto()
    "Zoom in slowly when threshold (based on variance) is reached."

    FAST = auto()
    "Zoom in fast when threshold (based on variance) is reached."

    SLOW_WHEN_ALIGNED = auto()
    """Zoom in slowly when threshold (based on variance) is reached
    and target is vertically and horizontally aligned.
    """

    FAST_WHEN_ALIGNED = auto()
    """Zoom in fast when threshold (based on variance) is reached
    and target is vertically and horizontally aligned.
    """

    GRADUALLY = auto()
    """When threshold (based on variance) is reached,
    zoom in faster the nearer the target is to the destination center, i.e.
     
    - zoom in fast when target is vertically and horizontally aligned,
    - zoom in slowly when target is mostly vertically and horizontally aligned,
      i.e. the distance of the target box to the edge of the live view is
      at least 1.5 times its
      
      - width (distance to left/right edge) or
      - height (distance to top/bottom edge)
    - otherwise, do not zoom in
    """

    # TODO it might be beneficial to gradually zoom based on predicting the
    #  size of the target after zooming, i.e. don't just use a magic constant
    #  as mode GRADUALLY, but predict the size change based on zoom ratio or
    #  DistanceEstimator

    # TODO add mode similar to GRADUALLY, but with configurable ranges,
    #  e.g. zoom in slow, when distance is at least times 2.0 times its
    #  width/height, and fast, when distance is at least times 4.0 times its
    #  width/height. Different factors might be used for width and height.


class ConfigurableTrackingStrategy(SimpleTrackingStrategy):
    rotation_mode: TrackingStrategyRotationMode
    zoom_in_mode: TrackingStrategyZoomInMode

    def __init__(
            self,
            destination: Destination,
            image_size: ImageSize,
            max_allowed_speed: float = 1000):
        super().__init__(destination, image_size, max_allowed_speed)
        self.rotation_mode = TrackingStrategyRotationMode.QUADRATIC_TO_LINEAR
        self.zoom_in_mode = TrackingStrategyZoomInMode.SLOW_WHEN_ALIGNED

    def _is_xy_aligned(self, target: Box) -> bool:
        return self._destination.box.contains_point(target.center)

    def _is_in_slow_zoom_in_range(self, target: Box):
        slow_zoom_in_range = Box.from_center_and_size(
            self._destination.center,
            self._image_size.width - 3 * target.width,
            self._image_size.height - 3 * target.height)
        return slow_zoom_in_range.intersect(target).area() > 0

    def _update_zoom_speed(self, camera_speeds, target: Box):
        if target.height < self._destination.min_size_box.height:
            camera_speeds.zoom_speed = self._zoom_in(target)
        elif target.height > self._destination.max_size_box.height:
            camera_speeds.zoom_speed = ZoomSpeed.ZOOM_OUT_FAST
        else:
            camera_speeds.zoom_speed = ZoomSpeed.ZOOM_STOPPED

    def _zoom_in(self, target):
        if self.zoom_in_mode is TrackingStrategyZoomInMode.SLOW:
            return ZoomSpeed.ZOOM_IN_SLOW
        if self.zoom_in_mode is TrackingStrategyZoomInMode.FAST:
            return ZoomSpeed.ZOOM_IN_FAST
        if self.zoom_in_mode is TrackingStrategyZoomInMode.SLOW_WHEN_ALIGNED:
            if self._is_xy_aligned(target):
                return ZoomSpeed.ZOOM_IN_SLOW
            else:
                return ZoomSpeed.ZOOM_STOPPED
        if self.zoom_in_mode is TrackingStrategyZoomInMode.FAST_WHEN_ALIGNED:
            if self._is_xy_aligned(target):
                return ZoomSpeed.ZOOM_IN_FAST
            else:
                return ZoomSpeed.ZOOM_STOPPED
        if self.zoom_in_mode is TrackingStrategyZoomInMode.GRADUALLY:
            if self._is_xy_aligned(target):
                return ZoomSpeed.ZOOM_IN_FAST
            elif self._is_in_slow_zoom_in_range(target):
                return ZoomSpeed.ZOOM_IN_SLOW
            else:
                return ZoomSpeed.ZOOM_STOPPED
        logger.warning(f"unhandled zoom in mode {self.zoom_in_mode}")
        return ZoomSpeed.ZOOM_STOPPED

    def _get_speed_by_distance(
            self, target: float, destination: float, size: int) -> float:
        distance = target - destination
        abs_distance = abs(distance)
        max_distance = size / 2
        variance = self._destination.variance
        if abs_distance < variance:
            if self.rotation_mode is TrackingStrategyRotationMode.STOP:
                return 0
            if (self.rotation_mode
                    is TrackingStrategyRotationMode.QUADRATIC_TO_LINEAR):
                return (self.max_allowed_speed * distance * abs_distance) \
                       / (variance * max_distance)
        percentage_distance = abs_distance / max_distance
        if self.rotation_mode is TrackingStrategyRotationMode.QUADRATIC:
            speed = (percentage_distance ** 2) * self.max_allowed_speed
        else:
            speed = percentage_distance * self.max_allowed_speed
        speed = min(self.max_allowed_speed, speed)
        if distance < 0:
            speed = -speed
        return speed


class StopIfLostTrackingStrategy(TrackingStrategy):
    _destination: Destination
    _trackingStrategy: TrackingStrategy
    _slowDownTime: float
    _hasTargetBeenLost: bool
    _timeOfLoss: float

    def __init__(
            self,
            destination: Destination,
            tracking_strategy: TrackingStrategy,
            slow_down_time: float):
        self._destination = destination
        self._trackingStrategy = tracking_strategy
        self._slowDownTime = slow_down_time
        self._hasTargetBeenLost = False
        self._timeOfLoss = time.time()

    def update(self,
               camera_speeds: CameraSpeeds,
               target: Optional[Box],
               is_target_lost: bool) -> None:
        self._trackingStrategy.update(camera_speeds, target, is_target_lost)
        if is_target_lost:
            if not self._hasTargetBeenLost:
                self._timeOfLoss = time.time()
            else:
                delta_time = time.time() - self._timeOfLoss
                t = min(delta_time, self._slowDownTime)
                slow_down_factor = 1 - (t / self._slowDownTime)
                camera_speeds.pan_speed = \
                    camera_speeds.pan_speed * slow_down_factor
                camera_speeds.tilt_speed = \
                    camera_speeds.tilt_speed * slow_down_factor
                camera_speeds.zoom_speed = ZoomSpeed.ZOOM_STOPPED
        self._hasTargetBeenLost = is_target_lost


class AlignTrackingStrategy(TrackingStrategy):
    @abstractmethod
    def is_aligned(self, target: Box) -> bool:
        raise NotImplementedError


class SimpleAlignTrackingStrategy(SimpleTrackingStrategy,
                                  AlignTrackingStrategy):
    def is_aligned(self, target: Box) -> bool:
        return self._destination.box.contains_point(target.center)


class ConfigurableAlignTrackingStrategy(ConfigurableTrackingStrategy,
                                        AlignTrackingStrategy):

    def _is_zoom_aligned(self, target: Box) -> bool:
        min_height = self._destination.min_size_box.height
        max_height = self._destination.max_size_box.height
        return min_height <= target.height <= max_height

    def is_aligned(self, target: Box) -> bool:
        return self._is_xy_aligned(target) and self._is_zoom_aligned(target)


class ConfigurableTrackingStrategyUi:

    def __init__(
            self,
            tracking_strategy: ConfigurableTrackingStrategy,
            align_strategy: ConfigurableAlignTrackingStrategy) -> None:
        self._tracking_strategy = tracking_strategy
        self._align_strategy = align_strategy

    def on_change(self, _value, rotation_mode):
        logger.debug(f"change rotation mode to: {str(rotation_mode)}")
        self._tracking_strategy.rotation_mode = rotation_mode
        self._align_strategy.rotation_mode = rotation_mode

    def create_radio_button(
            self,
            name: str,
            value: TrackingStrategyRotationMode,
            initial_state=0):
        import cv2
        cv2.createButton(
            name,
            self.on_change,
            userData=value,
            buttonType=cv2.QT_RADIOBOX,
            initialButtonState=initial_state)

    def open(self) -> None:
        self.create_radio_button('Stop', TrackingStrategyRotationMode.STOP)
        self.create_radio_button(
            'Linear', TrackingStrategyRotationMode.LINEAR, initial_state=1)
        self.create_radio_button(
            'Quadratic', TrackingStrategyRotationMode.QUADRATIC)
        self.create_radio_button(
            'Quadratic-To-Linear',
            TrackingStrategyRotationMode.QUADRATIC_TO_LINEAR)

    def update(self) -> None:
        pass


class SearchTargetStrategy(Protocol):
    @abstractmethod
    def update(self, camera_speeds: CameraSpeeds) -> None:
        raise NotImplementedError


class RotateSearchTargetStrategy(SearchTargetStrategy):
    def __init__(self, speed=200):
        self.speed = speed

    def update(self, camera_speeds: CameraSpeeds) -> None:
        camera_speeds.pan_speed = self.speed
