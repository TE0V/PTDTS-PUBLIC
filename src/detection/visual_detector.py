"""
Visual detection using YOLO for PTDTS
Supports both PyTorch (.pt) and NCNN (.ncnn) formats with auto-detection
NCNN provides 2-3x faster inference on ARM64 (Raspberry Pi 5)
"""

import time
import logging
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Visual detection data"""
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]  # (x, y)
    confidence: float
    class_id: int
    class_name: str


class NCNNDetector:
    """NCNN backend for YOLO detection (optimized for ARM64)"""

    def __init__(self, model_path: str, confidence_threshold: float = 0.4):
        """
        Initialize NCNN detector

        Args:
            model_path: Path to .ncnn.param file (e.g., 'model.ncnn.param')
            confidence_threshold: Confidence threshold for detections
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.net = None
        self.class_names = []

    def load_model(self):
        """Load NCNN model"""
        try:
            import ncnn
            import yaml

            # Convert path to .param and .bin files
            param_path = self.model_path
            if param_path.endswith('.param'):
                bin_path = param_path.replace('.param', '.bin')
            else:
                param_path = self.model_path + '.param'
                bin_path = self.model_path + '.bin'

            logger.info(f"Loading NCNN model: param={param_path}, bin={bin_path}")

            # Create network
            self.net = ncnn.Net()
            self.net.opt.use_vulkan_compute = False  # CPU only for stability
            self.net.opt.num_threads = 4  # Use 4 cores on Pi 5

            # Load model
            self.net.load_param(param_path)
            self.net.load_model(bin_path)

            # Load class names from metadata.yaml if available
            metadata_path = Path(param_path).parent / "metadata.yaml"
            if metadata_path.exists():
                with open(metadata_path, 'r') as f:
                    metadata = yaml.safe_load(f)
                    if 'names' in metadata:
                        # Convert dict {0: 'class1', 1: 'class2'} to list ['class1', 'class2']
                        names_dict = metadata['names']
                        self.class_names = [names_dict[i] for i in sorted(names_dict.keys())]
                        logger.info(f"Loaded {len(self.class_names)} class names from metadata: {self.class_names}")

            # Fallback to .txt file if metadata doesn't have names
            if not self.class_names:
                names_path = param_path.replace('.param', '.txt')
                if Path(names_path).exists():
                    with open(names_path, 'r') as f:
                        self.class_names = [line.strip() for line in f.readlines()]
                    logger.info(f"Loaded {len(self.class_names)} class names from txt file")

            logger.info("NCNN model loaded successfully")

        except ImportError:
            logger.error("ncnn-python library not installed")
            raise
        except Exception as e:
            logger.error(f"Failed to load NCNN model: {e}")
            raise

    def detect(self, frame: np.ndarray, iou_threshold: float = 0.45) -> List[Detection]:
        """
        Perform detection using NCNN

        Args:
            frame: Input frame (RGB or BGR)
            iou_threshold: IoU threshold for NMS

        Returns:
            List of Detection objects
        """
        if self.net is None:
            self.load_model()

        try:
            import ncnn

            h, w = frame.shape[:2]

            # Prepare input (YOLOv11 expects 640x640)
            mat_in = ncnn.Mat.from_pixels_resize(
                frame,
                ncnn.Mat.PixelType.PIXEL_BGR,
                w, h,
                640, 640
            )

            # Normalize (YOLOv11 uses 0-1 range)
            mean_vals = [0.0, 0.0, 0.0]
            norm_vals = [1/255.0, 1/255.0, 1/255.0]
            mat_in.substract_mean_normalize(mean_vals, norm_vals)

            # Run inference
            ex = self.net.create_extractor()
            ex.input("in0", mat_in)

            ret, mat_out = ex.extract("out0")

            # Parse NCNN output (YOLOv11 format)
            detections = self._parse_ncnn_output(mat_out, w, h, iou_threshold)

            return detections

        except Exception as e:
            logger.error(f"NCNN detection error: {e}")
            return []

    def _parse_ncnn_output(self, mat_out, img_w: int, img_h: int, iou_threshold: float) -> List[Detection]:
        """Parse NCNN output tensor to detections"""
        detections = []

        try:
            # Convert ncnn.Mat to numpy array
            # YOLOv11 output shape: [num_classes+4, 8400]
            # Standard YOLO: [84, 8400] = 80 classes + 4 bbox coords
            # Custom UAV: [7, 8400] = 3 classes + 4 bbox coords
            output = np.array(mat_out).reshape(-1, mat_out.h)

            # Detect number of classes from tensor shape
            # After reshape, output is (8400, 7) for 3-class model
            # We need to read from shape[1] to get the feature dimension
            num_features = output.shape[1]  # e.g., 7 for UAV model, 84 for standard
            num_classes = num_features - 4  # Subtract 4 bbox coords

            logger.debug(f"NCNN output: shape={output.shape}, num_classes={num_classes}")

            # Transpose to [8400, num_features]
            output = output.T

            # Extract boxes and scores
            boxes = output[:, :4]  # [x_center, y_center, width, height]
            scores = output[:, 4:4+num_classes]  # Class scores (only read actual class columns!)

            # Get class with max score for each detection
            class_ids = np.argmax(scores, axis=1)
            confidences = np.max(scores, axis=1)

            # Filter by confidence
            mask = confidences > self.confidence_threshold
            boxes = boxes[mask]
            confidences = confidences[mask]
            class_ids = class_ids[mask]

            if len(boxes) == 0:
                return []

            # Convert from center format to corner format
            # Scale from 640x640 to original image size
            scale_x = img_w / 640.0
            scale_y = img_h / 640.0

            x_centers = boxes[:, 0] * scale_x
            y_centers = boxes[:, 1] * scale_y
            widths = boxes[:, 2] * scale_x
            heights = boxes[:, 3] * scale_y

            x1 = x_centers - widths / 2
            y1 = y_centers - heights / 2
            x2 = x_centers + widths / 2
            y2 = y_centers + heights / 2

            # Apply NMS
            indices = cv2.dnn.NMSBoxes(
                boxes.tolist(),
                confidences.tolist(),
                self.confidence_threshold,
                iou_threshold
            )

            if len(indices) > 0:
                for i in indices.flatten():
                    class_id = int(class_ids[i])
                    confidence = float(confidences[i])

                    bbox = (int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i]))
                    center = (int(x_centers[i]), int(y_centers[i]))

                    # Get class name
                    if class_id < len(self.class_names):
                        class_name = self.class_names[class_id]
                    else:
                        class_name = f"class_{class_id}"

                    detection = Detection(
                        bbox=bbox,
                        center=center,
                        confidence=confidence,
                        class_id=class_id,
                        class_name=class_name
                    )
                    detections.append(detection)

            # Sort by confidence
            detections.sort(key=lambda d: d.confidence, reverse=True)

        except Exception as e:
            logger.error(f"Error parsing NCNN output: {e}")

        return detections


class UltralyticsDetector:
    """Ultralytics backend for YOLO detection (PyTorch)"""

    def __init__(self, model_path: str, confidence_threshold: float = 0.4, device: str = "cpu"):
        """
        Initialize Ultralytics detector

        Args:
            model_path: Path to .pt model file
            confidence_threshold: Confidence threshold for detections
            device: Device for inference ('cpu' or 'cuda')
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.device = device
        self.model = None

    def load_model(self):
        """Load PyTorch model via Ultralytics"""
        try:
            from ultralytics import YOLO

            logger.info(f"Loading Ultralytics YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)

            # Set device
            if self.device == "cuda":
                try:
                    self.model.to('cuda')
                    logger.info("YOLO model loaded on CUDA")
                except:
                    logger.warning("CUDA not available, falling back to CPU")
                    self.device = "cpu"

            logger.info("Ultralytics YOLO model loaded successfully")

        except ImportError:
            logger.error("Ultralytics library not installed")
            raise
        except Exception as e:
            logger.error(f"Failed to load Ultralytics model: {e}")
            raise

    def detect(self, frame: np.ndarray, iou_threshold: float = 0.45) -> List[Detection]:
        """
        Perform detection using Ultralytics

        Args:
            frame: Input frame (RGB or BGR)
            iou_threshold: IoU threshold for NMS

        Returns:
            List of Detection objects
        """
        if self.model is None:
            self.load_model()

        try:
            # Run inference
            results = self.model.predict(
                frame,
                conf=self.confidence_threshold,
                iou=iou_threshold,
                verbose=False,
                device=self.device
            )

            # Parse results
            detections = []
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes

                for box in boxes:
                    # Extract data
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    # Calculate center
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Get class name
                    class_name = self.model.names.get(cls, f"class_{cls}")

                    # Create detection
                    detection = Detection(
                        bbox=(int(x1), int(y1), int(x2), int(y2)),
                        center=(center_x, center_y),
                        confidence=conf,
                        class_id=cls,
                        class_name=class_name
                    )
                    detections.append(detection)

                # Sort by confidence
                detections.sort(key=lambda d: d.confidence, reverse=True)

            return detections

        except Exception as e:
            logger.error(f"Ultralytics detection error: {e}")
            return []


class VisualDetector:
    """
    Multi-backend YOLO visual detector
    Auto-detects and uses appropriate backend based on model format:
    - .pt files -> Ultralytics (PyTorch)
    - .ncnn.param/.ncnn.bin files -> NCNN (optimized for ARM64)
    """

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.4,
        iou_threshold: float = 0.45,
        classes: Optional[list] = None,
        device: str = "cpu"
    ):
        """
        Initialize visual detector with auto-backend selection

        Args:
            model_path: Path to YOLO model file (.pt or .ncnn.param)
            confidence_threshold: Confidence threshold for detections
            iou_threshold: IoU threshold for NMS
            classes: List of class IDs to detect (None = all) - currently unused
            device: Device for inference ('cpu' or 'cuda') - only for PyTorch
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.classes = classes
        self.device = device

        # Detect backend based on file extension
        self.backend_type = self._detect_backend()
        self.backend = None
        self.is_loaded = False

        # Performance tracking
        self.inference_time = 0.0
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()

        logger.info(f"Visual detector initialized: model={model_path}, backend={self.backend_type}, conf={confidence_threshold}")

    def _detect_backend(self) -> str:
        """Detect which backend to use based on model path"""
        path = Path(self.model_path)

        if '.ncnn' in path.suffix or '.ncnn.param' in str(path):
            return 'ncnn'
        elif path.suffix == '.pt':
            return 'ultralytics'
        else:
            # Default to ultralytics for unknown extensions
            logger.warning(f"Unknown model format '{path.suffix}', defaulting to Ultralytics")
            return 'ultralytics'

    def load_model(self):
        """Load YOLO model using appropriate backend"""
        if self.is_loaded:
            return

        try:
            if self.backend_type == 'ncnn':
                self.backend = NCNNDetector(self.model_path, self.confidence_threshold)
            else:
                self.backend = UltralyticsDetector(self.model_path, self.confidence_threshold, self.device)

            self.backend.load_model()
            self.is_loaded = True

        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            raise

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Perform detection on frame

        Args:
            frame: Input frame (RGB or BGR)

        Returns:
            List of Detection objects
        """
        if not self.is_loaded:
            self.load_model()

        start_time = time.time()

        try:
            detections = self.backend.detect(frame, self.iou_threshold)

            # Update performance metrics
            self.inference_time = time.time() - start_time

            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count
                self.frame_count = 0
                self.last_fps_time = current_time

            return detections

        except Exception as e:
            logger.error(f"Error during detection: {e}")
            return []

    def get_fps(self) -> float:
        """Get current detection FPS"""
        return self.fps

    def get_inference_time(self) -> float:
        """Get last inference time in seconds"""
        return self.inference_time

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        show_labels: bool = True,
        show_confidence: bool = True,
        color: Tuple[int, int, int] = (0, 255, 255),  # Yellow in BGR
        thickness: int = 2
    ) -> np.ndarray:
        """
        Draw detections on frame

        Args:
            frame: Input frame (BGR)
            detections: List of detections
            show_labels: Show class labels
            show_confidence: Show confidence scores
            color: Bounding box color (BGR)
            thickness: Line thickness

        Returns:
            Annotated frame
        """
        annotated = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx, cy = det.center

            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            # Draw center point
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

            # Draw label
            if show_labels or show_confidence:
                label_parts = []
                if show_labels:
                    label_parts.append(det.class_name)
                if show_confidence:
                    label_parts.append(f"{det.confidence:.2f}")

                label = " ".join(label_parts)

                # Calculate label size and draw background
                (label_w, label_h), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                cv2.rectangle(
                    annotated,
                    (x1, y1 - label_h - 10),
                    (x1 + label_w, y1),
                    color,
                    -1
                )

                # Draw label text
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 0),
                    2
                )

        return annotated

    def draw_overlay(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        pan_angle: Optional[float] = None,
        tilt_angle: Optional[float] = None,
        fps: Optional[float] = None
    ) -> np.ndarray:
        """
        Draw complete overlay with detections and telemetry

        Args:
            frame: Input frame (BGR)
            detections: List of detections
            pan_angle: Current pan angle (degrees)
            tilt_angle: Current tilt angle (degrees)
            fps: Camera FPS

        Returns:
            Annotated frame with overlay
        """
        annotated = self.draw_detections(frame, detections)

        height, width = frame.shape[:2]

        # Draw center crosshair
        center_x, center_y = width // 2, height // 2
        crosshair_size = 30
        crosshair_color = (0, 255, 0)  # Green
        cv2.drawMarker(
            annotated,
            (center_x, center_y),
            crosshair_color,
            cv2.MARKER_CROSS,
            crosshair_size,
            2
        )

        # Draw tracking lines to detections
        for det in detections:
            cx, cy = det.center
            cv2.line(annotated, (center_x, center_y), (cx, cy), (255, 0, 0), 2)

        # Draw telemetry text
        telemetry_lines = []

        if fps is not None:
            telemetry_lines.append(f"FPS: {fps:.1f}")

        if pan_angle is not None:
            telemetry_lines.append(f"Pan: {pan_angle:.1f}°")

        if tilt_angle is not None:
            telemetry_lines.append(f"Tilt: {tilt_angle:.1f}°")

        telemetry_lines.append(f"Detections: {len(detections)}")

        if detections:
            best = detections[0]
            telemetry_lines.append(f"Best: {best.class_name} ({best.confidence:.2f})")

        # Draw telemetry
        y_offset = 30
        for line in telemetry_lines:
            cv2.putText(
                annotated,
                line,
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            y_offset += 30

        return annotated

    def close(self):
        """Clean up resources"""
        if self.backend is not None:
            if hasattr(self.backend, 'model'):
                del self.backend.model
            if hasattr(self.backend, 'net'):
                del self.backend.net
            self.backend = None
            self.is_loaded = False
        logger.info("Visual detector closed")
