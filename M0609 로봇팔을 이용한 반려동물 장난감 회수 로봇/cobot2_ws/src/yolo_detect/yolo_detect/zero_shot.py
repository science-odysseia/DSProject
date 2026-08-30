"""Zero-shot 부위 분할 폴백 — Grounding DINO로 박스를 찾고 SAM2로 마스크를 만든다."""
import time

import cv2
import numpy as np

from yolo_detect import config


class ZeroShotSegmenter:
    """학습 경로가 부위를 못 찾았을 때만 쓰는 예비 분할기. 모델은 첫 호출에 지연 로드한다."""

    def __init__(self, device=None, log=print):
        self.device = device or config.MASK_DEVICE
        self.log = log
        self._gdino_processor = None
        self._gdino_model = None
        self._sam2_processor = None
        self._sam2_model = None

    @property
    def loaded(self):
        return self._gdino_model is not None or self._sam2_model is not None

    def _ensure_gdino(self):
        if self._gdino_model is not None:
            return
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        t0 = time.time()
        self._gdino_processor = AutoProcessor.from_pretrained(config.ZERO_SHOT_GDINO_MODEL)
        self._gdino_model = AutoModelForZeroShotObjectDetection.from_pretrained(
            config.ZERO_SHOT_GDINO_MODEL).to(self.device)
        self._gdino_model.eval()
        self.log(f"[zeroshot] Grounding DINO 로드 {time.time() - t0:.1f}초 "
                 f"({config.ZERO_SHOT_GDINO_MODEL})")

    def _ensure_sam2(self):
        if self._sam2_model is not None:
            return
        from transformers import Sam2Processor, Sam2Model
        t0 = time.time()
        self._sam2_processor = Sam2Processor.from_pretrained(config.ZERO_SHOT_SAM2_MODEL)
        self._sam2_model = Sam2Model.from_pretrained(config.ZERO_SHOT_SAM2_MODEL).to(self.device)
        self._sam2_model.eval()
        self.log(f"[zeroshot] SAM2 로드 {time.time() - t0:.1f}초 ({config.ZERO_SHOT_SAM2_MODEL})")

    def mask_from_box(self, frame_bgr, box):
        """박스를 힌트로 SAM2 마스크를 만든다. 후보 중 IoU 최고를 채택한다."""
        import torch
        from PIL import Image
        self._ensure_sam2()
        pil = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        x1, y1, x2, y2 = [float(v) for v in box]
        inputs = self._sam2_processor(
            images=pil, input_boxes=[[[x1, y1, x2, y2]]], return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self._sam2_model(**inputs)
        masks = self._sam2_processor.post_process_masks(
            outputs.pred_masks, inputs["original_sizes"])[0]
        m = masks.squeeze(0) if masks.ndim == 4 else masks
        m = m.cpu().numpy()
        iou = outputs.iou_scores.squeeze().cpu().numpy().reshape(-1)
        best = int(np.argmax(iou)) if iou.size == m.shape[0] else 0
        return m[best].astype(bool), float(np.max(iou))

    def detect_part(self, frame_bgr, part, object_class):
        """'the <부위> of the <물체>' 프롬프트로 박스를 찾고 마스크까지 만든다."""
        import torch
        from PIL import Image
        self._ensure_gdino()
        prompt = f"the {part} of the {object_class}.".lower()
        pil = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        h, w = frame_bgr.shape[:2]
        inputs = self._gdino_processor(images=pil, text=prompt, return_tensors="pt").to(self.device)
        t0 = time.time()
        with torch.no_grad():
            outputs = self._gdino_model(**inputs)
        try:
            results = self._gdino_processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, threshold=config.ZERO_SHOT_GDINO_BOX_THR,
                text_threshold=config.ZERO_SHOT_GDINO_TEXT_THR, target_sizes=[(h, w)])
        except TypeError:
            results = self._gdino_processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, box_threshold=config.ZERO_SHOT_GDINO_BOX_THR,
                text_threshold=config.ZERO_SHOT_GDINO_TEXT_THR, target_sizes=[(h, w)])
        r = results[0]
        n = len(r["boxes"])
        self.log(f"[zeroshot] GDINO {time.time() - t0:.2f}초 '{prompt}' 탐지 {n}개")
        if n == 0:
            return None

        scores = np.asarray(r["scores"].cpu() if hasattr(r["scores"], "cpu") else r["scores"])
        best = int(np.argmax(scores))
        box = tuple(float(v) for v in r["boxes"][best].tolist())
        mask, iou = self.mask_from_box(frame_bgr, box)
        self.log(f"[zeroshot] '{part}' 박스 점수 {float(scores[best]):.2f} → "
                 f"SAM2 마스크 IoU {iou:.2f}, {int(mask.sum())}픽셀")
        return {"part": part, "mask": mask, "score": float(scores[best]), "box": box}
