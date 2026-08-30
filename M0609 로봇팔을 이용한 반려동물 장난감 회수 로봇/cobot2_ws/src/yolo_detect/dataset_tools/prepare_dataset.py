"""정찰(one_stage_node) YOLO11 학습용 데이터셋 준비 스크립트 — 로컬에서 1회 실행."""
import os
import random
import shutil
import xml.etree.ElementTree as ET
import zipfile

RESOURCE_DIR = os.path.join(os.path.dirname(__file__), "..", "resource")
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "dataset")
CLASSES = ["bag", "cube", "doll", "duck", "robot"]  # 이 순서가 곧 class index 0~4
VAL_RATIO = 0.2
SEED = 42

def voc_to_yolo_line(xml_path, class_to_idx):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    size = root.find("size")
    img_w = float(size.find("width").text)
    img_h = float(size.find("height").text)

    lines = []
    for obj in root.findall("object"):
        name = obj.find("name").text.strip()
        if name not in class_to_idx:
            raise ValueError(f"알 수 없는 클래스명 '{name}' in {xml_path}")
        box = obj.find("bndbox")
        xmin = float(box.find("xmin").text)
        ymin = float(box.find("ymin").text)
        xmax = float(box.find("xmax").text)
        ymax = float(box.find("ymax").text)

        cx = ((xmin + xmax) / 2.0) / img_w
        cy = ((ymin + ymax) / 2.0) / img_h
        w = (xmax - xmin) / img_w
        h = (ymax - ymin) / img_h
        lines.append(f"{class_to_idx[name]} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines

def main():
    random.seed(SEED)
    class_to_idx = {name: i for i, name in enumerate(CLASSES)}

    for split in ("train", "val"):
        os.makedirs(os.path.join(OUTPUT_DIR, "images", split), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_DIR, "labels", split), exist_ok=True)

    total_counts = {"train": 0, "val": 0}

    for cls in CLASSES:
        img_dirs = [
            os.path.join(RESOURCE_DIR, f"{cls}_data_set"),
            os.path.join(RESOURCE_DIR, cls),
        ]
        xml_dir = os.path.join(RESOURCE_DIR, f"{cls}_labeld")

        stem_to_img_dir = {}
        for img_dir in img_dirs:
            for f in os.listdir(img_dir):
                if f.lower().endswith(".jpg"):
                    stem = f[:-4]
                    if stem in stem_to_img_dir:
                        raise ValueError(
                            f"중복 파일명 '{stem}.jpg' — {stem_to_img_dir[stem]} 와 {img_dir} 양쪽에 존재"
                        )
                    stem_to_img_dir[stem] = img_dir

        stems = sorted(stem_to_img_dir.keys())
        random.shuffle(stems)

        n_val = round(len(stems) * VAL_RATIO)
        split_stems = {"val": stems[:n_val], "train": stems[n_val:]}

        for split, split_list in split_stems.items():
            for stem in split_list:
                src_img = os.path.join(stem_to_img_dir[stem], f"{stem}.jpg")
                src_xml = os.path.join(xml_dir, f"{stem}.xml")

                # 파일명이 서로 다른 클래스끼리 겹칠 일은 없지만(좌표 기반 파일명),
                # 명시적으로 클래스 접두어를 붙여 어느 클래스 원본인지 항상 구분 가능하게 함
                dst_stem = f"{cls}_{stem}"
                dst_img = os.path.join(OUTPUT_DIR, "images", split, f"{dst_stem}.jpg")
                dst_label = os.path.join(OUTPUT_DIR, "labels", split, f"{dst_stem}.txt")

                shutil.copyfile(src_img, dst_img)
                lines = voc_to_yolo_line(src_xml, class_to_idx)
                with open(dst_label, "w") as f:
                    f.write("\n".join(lines) + "\n")

                total_counts[split] += 1

    data_yaml_path = os.path.join(OUTPUT_DIR, "data.yaml")
    with open(data_yaml_path, "w") as f:
        f.write(
            "# yolo11 학습용 — prepare_dataset.py가 자동 생성\n"
            "# Colab에서는 이 파일의 path를 업로드한 dataset 폴더 경로로 바꿔서 사용\n"
            "path: .\n"
            "train: images/train\n"
            "val: images/val\n"
            f"nc: {len(CLASSES)}\n"
            f"names: {CLASSES}\n"
        )

    zip_path = OUTPUT_DIR + ".zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(OUTPUT_DIR):
            for name in files:
                full = os.path.join(root, name)
                arcname = os.path.relpath(full, os.path.dirname(OUTPUT_DIR))
                zf.write(full, arcname)

    print(f"train: {total_counts['train']}장, val: {total_counts['val']}장")
    print(f"dataset 폴더: {OUTPUT_DIR}")
    print(f"data.yaml: {data_yaml_path}")
    print(f"업로드용 zip: {zip_path}")

if __name__ == "__main__":
    main()
