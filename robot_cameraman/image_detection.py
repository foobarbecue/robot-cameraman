from abc import abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import PIL.Image
import PIL.ImageFont
from typing_extensions import Protocol

from robot_cameraman.box import Box


@dataclass
class DetectionCandidate:
    label_id: int
    score: float
    bounding_box: Box


class DetectionEngine(Protocol):
    @abstractmethod
    def detect(self, image) -> Iterable[DetectionCandidate]:
        raise NotImplementedError


class EdgeTpuDetectionEngine(DetectionEngine):
    def __init__(
            self,
            model: Path,
            confidence: float,
            max_objects: int) -> None:
        import edgetpu.detection.engine
        self._engine = edgetpu.detection.engine.DetectionEngine(str(model))
        self._confidence = confidence
        self._max_objects = max_objects

    def detect(self, image: PIL.Image.Image) -> Iterable[DetectionCandidate]:
        return map(
            lambda dc: DetectionCandidate(dc.label_id, dc.score,
                                          Box.from_points_iterable(
                                              dc.bounding_box)),
            self._engine.DetectWithImage(
                image,
                threshold=self._confidence,
                keep_aspect_ratio=True,
                relative_coord=False,
                top_k=self._max_objects))


class DummyDetectionEngine(DetectionEngine):
    def detect(self, image) -> Iterable[DetectionCandidate]:
        return []
