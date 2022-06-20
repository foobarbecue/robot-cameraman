import logging
from logging import Logger
from abc import abstractmethod
from typing_extensions import Protocol
from time import sleep

import simplebgc.gimbal
from simplebgc.commands import GetAnglesInCmd
from simplebgc.gimbal import ControlMode

logger: Logger = logging.getLogger(__name__)

# TODO interface should be independent from simplebgc module,
#   since other gimbals might use different modes or values
class Gimbal(Protocol):
    @abstractmethod
    def control(
            self,
            yaw_mode: ControlMode = ControlMode.speed,
            yaw_speed: float = 0,
            yaw_angle: float = 0,
            pitch_mode: ControlMode = ControlMode.speed,
            pitch_speed: float = 0,
            pitch_angle: float = 0,
            roll_mode: ControlMode = ControlMode.speed,
            roll_speed: float = 0,
            roll_angle: float = 0) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_angles(self) -> GetAnglesInCmd:
        raise NotImplementedError

class BescorGimbal(Gimbal):
    """
    This gimbal class is for use with this combination of products:
     - Bescor MP-101 Pan & Tilt Head
     - Witmotion BWT901CL intertial motion unit
     - Relay 4 Zero 3V 4 Channel Relay Shield for Raspberry Pi
    """
    def __init__(self):
        from gpiozero import LED
        from witmotion import IMU
        self.deadband: float = 2
        self.yaw_relays = [LED("BOARD31"), LED("BOARD33")]
        self.pitch_relays = [LED("BOARD35"), LED("BOARD37")]
        self.imu = IMU(baudrate=115200)
        self.update_interval: float = 0.1
        
    # Only supports angle mode
    def control(
            self,
            yaw_mode: ControlMode = ControlMode.angle,
            yaw_angle: float = 0,
            pitch_mode: ControlMode = ControlMode.angle,
            pitch_angle: float = 0) -> None:
        try:
            desired_yaw = yaw_angle
            desired_pitch = pitch_angle
            
            while True:
                # Probably not ok that this is blocking
                # Maybe shouldn't have a loop here
                sleep(0.1)
            
                z_angle = self.imu.get_angle()[2]
                z_err = z_angle - desired_yaw
                if abs(z_err) > self.deadband:
                    if z_err > 0:
                        self.yaw_relays[0].on()
                        logger.info(f'z error {z_err}, moving yaw0')
                    else:
                        self.yaw_relays[1].on()
                        logger.info(f'z error {z_err}, moving yaw1')
                else:
                    for relay in self.yaw_relays:
                        relay.off()
                    logger.info(f'z error {z_err}, within deadband')

                y_angle = self.imu.get_angle()[1]
                y_err = y_angle - desired_pitch
                if abs(y_err) > self.deadband:
                    if y_err > 0:
                        self.yaw_relays[0].on()
                        logger.info(f'y error {y_err}, moving pitch0')
                    else:
                        self.pitch_relays[1].on()
                        logger.info(f'y error {y_err}, moving pitch1')
                else:
                    for relay in self.pitch_relays:
                        relay.off()
                    logger.info(f'y error {y_err}, within deadband')
        finally:
            self.stop()

    def stop(self) -> None:
        for relay in self.yaw_relays + self.pitch_relays:
            relay.off()
    
    def get_angles(self):
        return self.imu.get_angle()

class SimpleBgcGimbal(simplebgc.gimbal.Gimbal, Gimbal):
    pass


class DummyGimbal(Gimbal):
    def control(self, yaw_mode: ControlMode = ControlMode.speed,
                yaw_speed: float = 0, yaw_angle: float = 0,
                pitch_mode: ControlMode = ControlMode.speed,
                pitch_speed: float = 0, pitch_angle: float = 0,
                roll_mode: ControlMode = ControlMode.speed,
                roll_speed: float = 0, roll_angle: float = 0) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_angles(self) -> GetAnglesInCmd:
        return GetAnglesInCmd(
            imu_angle_1=0,
            target_angle_1=0,
            target_speed_1=0,
            imu_angle_2=0,
            target_angle_2=0,
            target_speed_2=0,
            imu_angle_3=0,
            target_angle_3=0,
            target_speed_3=0)
