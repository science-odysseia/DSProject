#!/usr/bin/env python3
"""검출 창 뷰어 — Isaac 이 /dev/shm 에 쓴 프레임을 띄운다.

🚨 **별도 프로세스여야 한다.** Isaac 내장 cv2 는 headless 빌드라 `imshow`
   가 없고(tkinter·PyQt5 도 없다), 시스템 python3 의 cv2 는 GUI 를 지원한다.

  isaac:  ./run_v13.sh floor2 --detect
  뷰어:   python3 tools/detect_view.py        ← 시스템 python3 로 실행
"""
import os
import sys
import time

import cv2

PATH = sys.argv[1] if len(sys.argv) > 1 else "/dev/shm/cobot3_detect.jpg"
SCALE = float(os.environ.get("DETECT_SCALE", 2.0))
WIN = "cobot3 detect"
print(f"[뷰어] {PATH} 감시 — 창에서 q 로 종료")
last, shown = 0.0, False
while True:
    try:
        m = os.path.getmtime(PATH)
        if m != last:
            img = cv2.imread(PATH)
            if img is not None:
                # 🎯 2배 확대 (2026-08-11 사용자 요청). 프레임 자체는
                #    640x360 그대로 두고 **표시만** 키운다 — 렌더 비용
                #    변화 없음. SCALE 로 조절.
                if SCALE != 1.0:
                    img = cv2.resize(
                        img, None, fx=SCALE, fy=SCALE,
                        interpolation=cv2.INTER_NEAREST)
                cv2.imshow(WIN, img)
                last, shown = m, True
    except OSError:
        pass
    if cv2.waitKey(30) & 0xFF == ord("q"):
        break
    if not shown:
        time.sleep(0.1)
cv2.destroyAllWindows()
