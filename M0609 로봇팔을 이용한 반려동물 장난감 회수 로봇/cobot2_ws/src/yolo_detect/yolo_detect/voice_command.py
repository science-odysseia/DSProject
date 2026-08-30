"""음성 지시 입력."""

import os
import re
from collections import namedtuple

import pyaudio
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv

from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

from yolo_detect import config

OBJECT_CLASSES = ("bag", "cube", "doll", "duck", "robot")
PART_CLASSES = tuple(sorted(config.TWO_STAGE_PART_CLASSES))

_CLASS_SYNONYMS = (
    ("오리", "duck"), ("덕", "duck"), ("duck", "duck"),
    ("곰인형", "doll"), ("인형", "doll"), ("doll", "doll"),
    ("로봇", "robot"), ("robot", "robot"),
    ("정육면체", "cube"), ("큐브", "cube"), ("블록", "cube"), ("cube", "cube"),
    ("가방", "bag"), ("백팩", "bag"), ("bag", "bag"),
)
_PART_SYNONYMS = (
    ("머리", "head"), ("얼굴", "head"), ("head", "head"),
    ("몸통", "body"), ("바디", "body"), ("몸", "body"), ("배", "body"), ("body", "body"),
    ("다리", "leg"), ("발", "leg"), ("leg", "leg"),
    ("팔", "arm"), ("손", "arm"), ("arm", "arm"),
)
_RETRIEVAL_HINTS = ("가져", "집어", "회수", "잡아", "꺼내", "가지고")

Command = namedtuple("Command", ["raw_text", "object_class", "part"])

def _env_candidates():
    """OPENAI_API_KEY를 담은 .env를 찾을 경로 목록."""
    paths = [os.path.join(config.RESOURCE_PATH, ".env")]
    try:
        paths.append(os.path.join(
            get_package_share_directory("voice_processing"), "resource", ".env"))
    except Exception:
        pass
    return paths


def _load_api_key():
    """환경변수 → yolo_detect/resource/.env → voice_processing/resource/.env 순으로 키를 찾음."""
    key = os.getenv("OPENAI_API_KEY")
    if key:
        return key.strip()
    for path in _env_candidates():
        if not os.path.exists(path):
            continue
        load_dotenv(dotenv_path=path)
        key = os.getenv("OPENAI_API_KEY")
        if key:
            return key.strip()
    raise RuntimeError(
        "OPENAI_API_KEY가 없습니다 — 환경변수로 넣거나 다음 중 하나에 .env를 두세요: "
        + ", ".join(_env_candidates()))

def parse_by_keyword(text):
    """발화에서 키워드로 물체와 부위를 뽑음."""
    low = (text or "").strip().lower()
    obj = next((v for k, v in _CLASS_SYNONYMS if k in low), None)
    part = next((v for k, v in _PART_SYNONYMS if k in low), None)
    return obj, part

def parse_by_llm(text, llm):
    """발화를 LLM으로 해석해 물체와 부위를 뽑음."""
    from langchain.prompts import PromptTemplate

    prompt = PromptTemplate(
        input_variables=["text", "classes", "parts"],
        template=(
            "너는 로봇 회수 명령 파서다. 발화에서 회수할 물체와 부위를 뽑아라.\n"
            "물체는 반드시 [{classes}] 중 하나, 부위는 반드시 [{parts}] 중 하나여야 한다.\n"
            "발화에 없으면 그 자리에 none을 쓴다. 다른 말은 절대 붙이지 마라.\n"
            "출력 형식(정확히 한 줄): <물체>|<부위>\n\n"
            "예: '오리 몸통 집어와' -> duck|body\n"
            "예: '인형 다리 가져와줘' -> doll|leg\n"
            "예: '정찰해' -> none|none\n\n"
            "발화: {text}\n"
            "출력: "
        ),
    )
    raw = llm.invoke(prompt.format(
        text=text, classes=", ".join(OBJECT_CLASSES),
        parts=", ".join(PART_CLASSES))).content
    m = re.search(r"([a-z]+)\s*\|\s*([a-z]+)", str(raw).strip().lower())
    if not m:
        return None, None
    obj = m.group(1) if m.group(1) in OBJECT_CLASSES else None
    part = m.group(2) if m.group(2) in PART_CLASSES else None
    return obj, part

def parse_command(text, llm=None, log=None):
    """발화 한 줄 -> Command."""
    obj = part = None
    if llm is not None:
        try:
            obj, part = parse_by_llm(text, llm)
            if log:
                log(f"  LLM 해석: 물체={obj} 부위={part}")
        except Exception as e:
            if log:
                log(f"  ⚠️ LLM 해석 실패({type(e).__name__}: {e}) — 키워드 폴백")
    if obj is None or part is None:
        k_obj, k_part = parse_by_keyword(text)
        if obj is None and k_obj is not None:
            obj = k_obj
            if log:
                log(f"  키워드 보정: 물체={obj}")
        if part is None and k_part is not None:
            part = k_part
            if log:
                log(f"  키워드 보정: 부위={part}")
    return Command(raw_text=text, object_class=obj, part=part)

def has_retrieval_intent(text):
    """회수 의도 표현이 있는지."""
    return any(h in (text or "") for h in _RETRIEVAL_HINTS)

class VoiceCommandListener:
    """웨이크워드 대기 -> 녹음 -> STT -> 의도 해석."""

    def __init__(self, log=None, use_llm=True):
        self.log = log or (lambda msg: print(msg))
        api_key = _load_api_key()
        self.stt = STT(openai_api_key=api_key)
        self.llm = None
        if use_llm:
            try:
                from langchain_openai import ChatOpenAI
                self.llm = ChatOpenAI(model="gpt-4o", temperature=0.0,
                                      openai_api_key=api_key)
            except Exception as e:
                self.log(f"⚠️ LLM 초기화 실패({type(e).__name__}: {e}) — 키워드 해석만 사용")

        self.mic_config = MicConfig(
            chunk=config.MIC_CHUNK, rate=config.MIC_RATE,
            channels=config.MIC_CHANNELS, record_seconds=config.MIC_RECORD_SECONDS,
            fmt=pyaudio.paInt16, device_index=config.MIC_DEVICE_INDEX,
            buffer_size=config.MIC_BUFFER_SIZE)
        self.mic = MicController(config=self.mic_config)
        self.wakeup_word = WakeupWord(self.mic_config.buffer_size)

    def _wait_for_wakeword(self):
        self.mic.open_stream()
        self.wakeup_word.set_stream(self.mic.stream)
        self.log("🎤 웨이크워드 대기 중 — 'Hello Rokey' 라고 말하세요")
        try:
            while not self.wakeup_word.is_wakeup():
                pass
        finally:
            self.mic.close_stream()
        self.log("✅ 웨이크워드 감지")

    def listen(self, max_attempts=5):
        """지시가 성립할 때까지 최대 max_attempts번 들음."""
        for attempt in range(1, max_attempts + 1):
            self._wait_for_wakeword()
            try:
                text = self.stt.speech2text()
            except Exception as e:
                self.log(f"⚠️ STT 실패({type(e).__name__}: {e}) — 다시 듣습니다")
                continue
            self.log(f"📝 발화({attempt}/{max_attempts}): \"{text}\"")

            cmd = parse_command(text, llm=self.llm, log=self.log)
            if cmd.part is None:
                hint = "" if has_retrieval_intent(text) else " (회수 의도도 안 보임)"
                self.log(
                    f"⚠️ 잡을 부위를 못 알아들었습니다{hint} — "
                    f"'오리 몸통 집어와' 처럼 물체와 부위를 함께 말해주세요. "
                    f"부위 어휘: {', '.join(PART_CLASSES)}")
                continue

            self.log(f"🎯 지시 확정: 물체={cmd.object_class or '(미지정)'} "
                     f"부위={cmd.part}")
            return cmd

        self.log(f"❌ {max_attempts}번 시도했지만 지시를 못 받았습니다")
        return None
