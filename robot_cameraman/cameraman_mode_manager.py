import logging
from logging import Logger
from typing import Optional

from robot_cameraman.box import Box
from robot_cameraman.camera_controller import CameraController
from robot_cameraman.tracking import TrackingStrategy, CameraSpeeds

logger: Logger = logging.getLogger(__name__)


class CameramanModeManager:

    def __init__(
            self,
            camera_controller: CameraController,
            tracking_strategy: TrackingStrategy) -> None:
        self._camera_controller = camera_controller
        self._tracking_strategy = tracking_strategy
        self._camera_speeds: CameraSpeeds = CameraSpeeds()
        self._is_manual_mode = False

    def update(self, target: Optional[Box] = None) -> None:
        if not self._is_manual_mode:
            if target is None:
                # search target
                self._camera_speeds.pan_speed = 200
            else:
                self._tracking_strategy.update(self._camera_speeds, target)
        self._camera_controller.update(self._camera_speeds)

    def start(self):
        self._camera_controller.start()

    def stop(self) -> None:
        if self._camera_controller:
            logger.debug('Stop camera')
            self._camera_controller.stop()

    def manual_mode(self) -> None:
        self._is_manual_mode = True

    def manual_rotate(self, pan_speed: int) -> None:
        self._camera_speeds.pan_speed = pan_speed