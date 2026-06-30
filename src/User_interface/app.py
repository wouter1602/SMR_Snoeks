import csv
import io
import json
import os
import subprocess
import re
import sys
import threading
import uuid
from datetime import datetime

from flask import Flask, jsonify, render_template, request
from flask_cors import CORS

app = Flask(__name__)
import threading
TRANSPORT_STATUS_LOCK = threading.Lock()
CORS(app)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")
PARTS_FILE = os.path.join(DATA_DIR, "parts_catalog.json")
BATCHES_FILE = os.path.join(DATA_DIR, "batches.json")
ACTIVE_BATCH_FILE = os.path.join(DATA_DIR, "active_batch.json")
TRANSPORT_STATUS_FILE = os.path.join(DATA_DIR, "transport_status.json")
TRANSPORT_SCRIPT = os.path.join(BASE_DIR, "transport.py")
MAIN_CONTROL_SCRIPT = os.path.join(BASE_DIR, "main_control_v3.py")

DEFAULT_PARTS = [
    {"id": 1, "name": "Bout M4 x 10mm", "article": "BT-M4-010", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 2, "name": "Bout M4 x 16mm", "article": "BT-M4-016", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 3, "name": "Bout M4 x 20mm", "article": "BT-M4-020", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 4, "name": "Bout M5 x 10mm", "article": "BT-M5-010", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 5, "name": "Bout M5 x 16mm", "article": "BT-M5-016", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 6, "name": "Bout M5 x 20mm", "article": "BT-M5-020", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 7, "name": "Bout M6 x 10mm", "article": "BT-M6-010", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 8, "name": "Bout M6 x 16mm", "article": "BT-M6-016", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 9, "name": "Bout M6 x 20mm", "article": "BT-M6-020", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 10, "name": "Bout M6 x 25mm", "article": "BT-M6-025", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 11, "name": "Bout M8 x 20mm", "article": "BT-M8-020", "allowedHopperTypes": ["largeBolts"], "handlingMethod": "hopper"},
    {"id": 12, "name": "Bout M8 x 25mm", "article": "BT-M8-025", "allowedHopperTypes": ["largeBolts"], "handlingMethod": "hopper"},
    {"id": 13, "name": "Moer M4", "article": "MR-M4-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 14, "name": "Moer M5", "article": "MR-M5-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 15, "name": "Moer M6", "article": "MR-M6-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 16, "name": "Moer M8", "article": "MR-M8-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 17, "name": "Borgmoer M4", "article": "BM-M4-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 18, "name": "Borgmoer M5", "article": "BM-M5-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 19, "name": "Borgmoer M6", "article": "BM-M6-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 20, "name": "Borgmoer M8", "article": "BM-M8-000", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 21, "name": "Vlakke ring 4mm", "article": "RG-4-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 22, "name": "Vlakke ring 5mm", "article": "RG-5-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 23, "name": "Vlakke ring 6mm", "article": "RG-6-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 24, "name": "Vlakke ring 8mm", "article": "RG-8-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 25, "name": "Veerring 4mm", "article": "VR-4-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 26, "name": "Veerring 5mm", "article": "VR-5-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 27, "name": "Veerring 6mm", "article": "VR-6-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 28, "name": "Veerring 8mm", "article": "VR-8-000", "allowedHopperTypes": ["washers"], "handlingMethod": "hopper"},
    {"id": 29, "name": "Inbusbout M4 x 10mm", "article": "IB-M4-010", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 30, "name": "Inbusbout M4 x 16mm", "article": "IB-M4-016", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 31, "name": "Inbusbout M5 x 10mm", "article": "IB-M5-010", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 32, "name": "Inbusbout M5 x 16mm", "article": "IB-M5-016", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 33, "name": "Inbusbout M6 x 10mm", "article": "IB-M6-010", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 34, "name": "Inbusbout M6 x 16mm", "article": "IB-M6-016", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 35, "name": "Schroef M3 x 8mm", "article": "SC-M3-008", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 36, "name": "Schroef M3 x 12mm", "article": "SC-M3-012", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 37, "name": "Schroef M4 x 8mm", "article": "SC-M4-008", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 38, "name": "Schroef M4 x 12mm", "article": "SC-M4-012", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 39, "name": "Schroef M4 x 16mm", "article": "SC-M4-016", "allowedHopperTypes": ["smallBolts"], "handlingMethod": "hopper"},
    {"id": 40, "name": "Schroef M5 x 10mm", "article": "SC-M5-010", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 41, "name": "Schroef M5 x 16mm", "article": "SC-M5-016", "allowedHopperTypes": ["mediumBolts"], "handlingMethod": "hopper"},
    {"id": 42, "name": "Clip 15mm", "article": "CL-015-00", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 43, "name": "Clip 20mm", "article": "CL-020-00", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 44, "name": "Clip 25mm", "article": "CL-025-00", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 45, "name": "Pen 3 x 20mm", "article": "PN-3-020", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 46, "name": "Pen 4 x 20mm", "article": "PN-4-020", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 47, "name": "Pen 5 x 30mm", "article": "PN-5-030", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 48, "name": "Popnagel 4.8mm", "article": "PN-48-00", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
    {"id": 49, "name": "Kabelclip 8mm", "article": "KC-008-00", "allowedHopperTypes": [], "handlingMethod": "robotarm"},
]

SLOT_ROWS = ["A", "B", "C"]
SLOT_COLUMNS = [1, 2, 3, 4]
SLOTS = [f"{row}{col}" for row in SLOT_ROWS for col in SLOT_COLUMNS]
HOPPER_LAYOUT = {
    "smallBolts": [f"S-{index}" for index in range(1, 6)],
    "mediumBolts": [f"M-{index}" for index in range(1, 6)],
    "largeBolts": [f"L-{index}" for index in range(1, 6)],
    "washers": [f"W-{index}" for index in range(1, 11)],
}
HOPPER_TYPE_OPTIONS = ["smallBolts", "mediumBolts", "largeBolts", "washers"]
CSV_SET_PATTERN = re.compile(r"\bP\d{9}\b", re.IGNORECASE)
CSV_ARTICLE_PATTERN = re.compile(r"^\s*([A-Z]?\d{5,}|P\d{9})\s*-\s*(.+)$", re.IGNORECASE)


def infer_part_handling(part):
    name = (part.get("name") or "").lower()
    article = (part.get("article") or "").upper()

    if "ring" in name or "washer" in name or article.startswith(("RG-", "VR-")):
        return ["washers"], "hopper"
    if any(token in name for token in ["bout", "schroef"]) or article.startswith(("BT-", "IB-", "SC-")):
        if "M8" in name or "-M8-" in article:
            return ["largeBolts"], "hopper"
        if "M5" in name or "M6" in name or "-M5-" in article or "-M6-" in article:
            return ["mediumBolts"], "hopper"
        return ["smallBolts"], "hopper"
    return [], "robotarm"


def normalize_weight_grams(value):
    if value in (None, ""):
        return None
    try:
        weight = float(value)
    except (TypeError, ValueError):
        return None
    if weight < 0:
        return None
    rounded = round(weight, 3)
    return int(rounded) if rounded.is_integer() else rounded


def normalize_part(part):
    allowed_hopper_types = part.get("allowedHopperTypes")
    handling_method = part.get("handlingMethod")
    if allowed_hopper_types is None or handling_method is None:
        inferred_types, inferred_method = infer_part_handling(part)
        allowed_hopper_types = inferred_types if allowed_hopper_types is None else allowed_hopper_types
        handling_method = inferred_method if handling_method is None else handling_method

    allowed_hopper_types = [hopper_type for hopper_type in (allowed_hopper_types or []) if hopper_type in HOPPER_TYPE_OPTIONS]
    if handling_method not in {"hopper", "robotarm"}:
        handling_method = "robotarm" if not allowed_hopper_types else "hopper"
    if handling_method == "robotarm":
        allowed_hopper_types = []
    weight_grams = normalize_weight_grams(part.get("weightGrams"))

    return {
        "id": part["id"],
        "name": part["name"],
        "article": part["article"],
        "allowedHopperTypes": allowed_hopper_types,
        "handlingMethod": handling_method,
        "weightGrams": weight_grams,
    }


def validate_part_payload(data, parts, current_part_id=None):
    name = (data.get("name") or "").strip()
    article = (data.get("article") or "").strip()
    handling_method = data.get("handlingMethod") or "hopper"
    allowed_hopper_types = [
        hopper_type
        for hopper_type in (data.get("allowedHopperTypes") or [])
        if hopper_type in HOPPER_TYPE_OPTIONS
    ]
    raw_weight = data.get("weightGrams")
    weight_grams = normalize_weight_grams(raw_weight)

    if not name:
        return None, "Naam is verplicht."
    if not article:
        return None, "Artikelnummer is verplicht."
    if handling_method not in {"hopper", "robotarm"}:
        return None, "Ongeldige handling methode."
    if handling_method == "hopper" and not allowed_hopper_types:
        return None, "Kies minimaal 1 hoppertype."
    if raw_weight not in (None, "") and weight_grams is None:
        return None, "Gewicht moet een geldig getal van 0 of hoger zijn."
    if any(part["article"].lower() == article.lower() and part["id"] != current_part_id for part in parts):
        return None, "Artikelnummer bestaat al."

    return {
        "name": name,
        "article": article,
        "allowedHopperTypes": allowed_hopper_types,
        "handlingMethod": handling_method,
        "weightGrams": weight_grams,
    }, None


def sync_part_details_in_batches(part):
    batches = load_batches()
    changed = False
    for batch in batches:
        for items in batch.get("slots", {}).values():
            for batch_part in items:
                if batch_part.get("partId") != part["id"]:
                    continue
                if batch_part.get("name") != part["name"]:
                    batch_part["name"] = part["name"]
                    changed = True
                if batch_part.get("article") != part["article"]:
                    batch_part["article"] = part["article"]
                    changed = True
    if changed:
        save_batches(batches)


def ensure_data_files():
    os.makedirs(DATA_DIR, exist_ok=True)

    if not os.path.exists(PARTS_FILE):
        with open(PARTS_FILE, "w", encoding="utf-8") as file:
            json.dump(DEFAULT_PARTS, file, indent=2, ensure_ascii=False)

    if not os.path.exists(BATCHES_FILE):
        with open(BATCHES_FILE, "w", encoding="utf-8") as file:
            json.dump([], file, indent=2, ensure_ascii=False)


def read_json(path, fallback):
    if not os.path.exists(path):
        return fallback
    try:
        with open(path, encoding="utf-8") as file:
            return json.load(file)
    except (json.JSONDecodeError, OSError) as err:
        print(f"[READ_JSON] {path} corrupt of onleesbaar ({err}); fallback gebruikt.")
        return fallback if not isinstance(fallback, (dict, list)) else type(fallback)(fallback) if isinstance(fallback, dict) else list(fallback)


def write_json(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temp_path = f"{path}.{uuid.uuid4().hex}.tmp"
    with open(temp_path, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
    os.replace(temp_path, path)


def load_parts():
    parts = [normalize_part(part) for part in read_json(PARTS_FILE, DEFAULT_PARTS)]
    return sorted(parts, key=lambda part: (part["name"].lower(), part["article"].lower()))


def save_parts(parts):
    write_json(PARTS_FILE, [normalize_part(part) for part in parts])


def empty_slots():
    return {slot_id: [] for slot_id in SLOTS}


def normalize_slots(slots):
    normalized = empty_slots()
    for slot_id, items in (slots or {}).items():
        if slot_id in normalized:
            normalized[slot_id] = items
    return normalized


def parse_number(value):
    if value in (None, ""):
        return None
    text = str(value).strip().replace(",", ".")
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    return int(number) if number.is_integer() else number


def clean_csv_cell(value):
    return (value or "").strip()


def extract_set_number(detail):
    matches = CSV_SET_PATTERN.findall(detail or "")
    return matches[0].upper() if matches else None


def parse_csv_part_detail(detail):
    match = CSV_ARTICLE_PATTERN.match(detail or "")
    if not match:
        return None, None
    return match.group(1).strip().upper(), match.group(2).strip()


def infer_import_handling(part_type, name):
    type_text = (part_type or "").lower()
    name_text = (name or "").lower()
    source = f"{type_text} {name_text}"

    if "washer" in source:
        return ["washers"], "hopper"
    if "bolt" in type_text and "bag" not in type_text:
        metric_match = re.search(r"\bm\s*(\d+)", name_text, re.IGNORECASE)
        metric_size = int(metric_match.group(1)) if metric_match else None
        if metric_size and metric_size >= 8:
            return ["largeBolts"], "hopper"
        if metric_size and metric_size >= 5:
            return ["mediumBolts"], "hopper"
        return ["smallBolts"], "hopper"
    return [], "robotarm"


def read_uploaded_csv(file_storage):
    raw = file_storage.read()
    if not raw:
        return []
    for encoding in ("utf-8-sig", "cp1252", "latin-1"):
        try:
            text = raw.decode(encoding)
            break
        except UnicodeDecodeError:
            text = None
    if text is None:
        text = raw.decode("utf-8", errors="replace")
    return list(csv.reader(io.StringIO(text)))


def find_csv_header(rows):
    for row_index, row in enumerate(rows):
        labels = {clean_csv_cell(value).lower(): index for index, value in enumerate(row)}
        if {"detail", "quantity", "position"}.issubset(labels):
            return {
                "rowIndex": row_index,
                "detail": labels["detail"],
                "quantity": labels["quantity"],
                "type": labels.get("type"),
                "position": labels["position"],
                "weight": labels.get("weight [g]"),
            }
    return None


def cell_at(row, index):
    if index is None or index >= len(row):
        return ""
    return clean_csv_cell(row[index])


def parse_installation_sets_csv(file_storage):
    rows = read_uploaded_csv(file_storage)
    header = find_csv_header(rows)
    if not header:
        return [], "Geen geldige CSV-kop gevonden. Verwacht kolommen Detail, Quantity en Position."

    sets = []
    current_set = None
    sets_by_number = {}

    for row in rows[header["rowIndex"] + 1 :]:
        detail = cell_at(row, header["detail"])
        if not detail:
            continue

        set_number = extract_set_number(detail)
        article, name = parse_csv_part_detail(detail)
        position = cell_at(row, header["position"]).upper()
        quantity = parse_number(cell_at(row, header["quantity"]))

        is_set_row = detail.upper().startswith("M ") and set_number
        if is_set_row:
            current_set = sets_by_number.get(set_number)
            if current_set is None:
                current_set = {
                    "setNumber": set_number,
                    "name": re.sub(r"^M\s+", "", detail, flags=re.IGNORECASE).strip(),
                    "parts": [],
                }
                sets_by_number[set_number] = current_set
                sets.append(current_set)
            continue

        if not current_set or not article or not name:
            continue
        if position not in SLOTS or quantity is None or quantity <= 0:
            continue

        part_type = cell_at(row, header["type"])
        weight_grams = normalize_weight_grams(cell_at(row, header["weight"]))
        allowed_hopper_types, handling_method = infer_import_handling(part_type, name)
        current_set["parts"].append(
            {
                "article": article,
                "name": name,
                "type": part_type,
                "position": position,
                "quantityPerSet": quantity,
                "weightGrams": weight_grams,
                "allowedHopperTypes": allowed_hopper_types,
                "handlingMethod": handling_method,
            }
        )

    parsed_sets = []
    for item in sets:
        if not item["parts"]:
            continue
        item["partCount"] = len(item["parts"])
        item["totalPerSet"] = sum(part["quantityPerSet"] for part in item["parts"])
        parsed_sets.append(item)

    if not parsed_sets:
        return [], "Geen onderdelen met geldige positie gevonden in de CSV."
    return parsed_sets, None


def ensure_import_parts(import_parts):
    parts = load_parts()
    parts_by_article = {part["article"].lower(): part for part in parts}
    next_id = max((part["id"] for part in parts), default=0) + 1
    created_count = 0
    updated_count = 0
    changed = False

    for import_part in import_parts:
        article = (import_part.get("article") or "").strip()
        name = (import_part.get("name") or "").strip()
        if not article or not name:
            continue

        key = article.lower()
        existing_part = parts_by_article.get(key)
        payload = {
            "name": name,
            "article": article,
            "allowedHopperTypes": import_part.get("allowedHopperTypes") or [],
            "handlingMethod": import_part.get("handlingMethod") or "robotarm",
            "weightGrams": import_part.get("weightGrams"),
        }

        if existing_part is None:
            new_part = normalize_part({"id": next_id, **payload})
            next_id += 1
            parts.append(new_part)
            parts_by_article[key] = new_part
            created_count += 1
            changed = True
            continue

        updated_part = normalize_part({"id": existing_part["id"], **payload})
        if any(existing_part.get(field) != updated_part.get(field) for field in ("name", "allowedHopperTypes", "handlingMethod", "weightGrams")):
            existing_part.update(updated_part)
            updated_count += 1
            changed = True

    if changed:
        save_parts(parts)

    return sorted(
        [normalize_part(part) for part in parts],
        key=lambda part: (part["name"].lower(), part["article"].lower()),
    ), created_count, updated_count


def build_import_slots(import_parts, batch_size):
    parts, created_count, updated_count = ensure_import_parts(import_parts)
    parts_by_article = {part["article"].lower(): part for part in parts}
    slots = empty_slots()

    for import_part in import_parts:
        slot_id = (import_part.get("position") or "").upper()
        if slot_id not in slots:
            continue
        catalog_part = parts_by_article.get((import_part.get("article") or "").lower())
        quantity_per_set = parse_number(import_part.get("quantityPerSet"))
        if not catalog_part or quantity_per_set is None or quantity_per_set <= 0:
            continue
        total_quantity = int(quantity_per_set)
        existing_item = next((item for item in slots[slot_id] if item["partId"] == catalog_part["id"]), None)
        if existing_item:
            existing_item["quantity"] += total_quantity
        else:
            slots[slot_id].append(
                {
                    "partId": catalog_part["id"],
                    "name": catalog_part["name"],
                    "article": catalog_part["article"],
                    "quantity": total_quantity,
                }
            )

    return slots, created_count, updated_count


def load_batches():
    batches = read_json(BATCHES_FILE, [])
    for batch in batches:
        batch["slots"] = normalize_slots(batch.get("slots"))
        batch["partCount"] = sum(len(items) for items in batch["slots"].values())
        batch["totalQty"] = sum(part.get("quantity", 0) for items in batch["slots"].values() for part in items)
    return batches


def save_batches(batches):
    for batch in batches:
        batch["slots"] = normalize_slots(batch.get("slots"))
    write_json(BATCHES_FILE, batches)


def build_batch_start_payload(batch):
    parts_by_id = {part["id"]: part for part in load_parts()}
    used_hoppers = set()
    part_hoppers = {}
    deliveries = []

    for slot_id in SLOTS:
        for item in batch.get("slots", {}).get(slot_id, []):
            catalog_part = parts_by_id.get(item.get("partId"), {})
            allowed_hopper_types = catalog_part.get("allowedHopperTypes") or []
            handling_method = catalog_part.get("handlingMethod") or "robotarm"
            part_key = item.get("partId") or item.get("article")
            hopper_id = None

            if handling_method == "hopper":
                hopper_id = part_hoppers.get(part_key)
                if hopper_id is None:
                    for hopper_type in allowed_hopper_types:
                        for candidate_id in HOPPER_LAYOUT.get(hopper_type, []):
                            if candidate_id not in used_hoppers:
                                hopper_id = candidate_id
                                used_hoppers.add(candidate_id)
                                part_hoppers[part_key] = candidate_id
                                break
                        if hopper_id:
                            break

            deliveries.append(
                {
                    "buffer": slot_id,
                    "hopper": hopper_id,
                    "handlingMethod": "hopper" if hopper_id else "robotarm",
                    "partId": item.get("partId"),
                    "name": item.get("name"),
                    "article": item.get("article"),
                    "deliverQuantity": int(item.get("quantity") or 0),
                    "weightGrams": catalog_part.get("weightGrams"),
                }
            )

    return {
        "status": "started",
        "startedAt": datetime.now().isoformat(timespec="seconds"),
        "batch": {
            "id": batch["id"],
            "name": batch["name"],
            "created": batch.get("created"),
            "batchSize": batch.get("batchSize"),
            "partCount": batch.get("partCount"),
            "totalQty": batch.get("totalQty"),
        },
        "deliveries": deliveries,
        "hopperAssignments": [
            {
                "hopper": delivery["hopper"],
                "buffer": delivery["buffer"],
                "partId": delivery["partId"],
                "article": delivery["article"],
                "deliverQuantity": delivery["deliverQuantity"],
            }
            for delivery in deliveries
            if delivery["hopper"]
        ],
        "robotarmDeliveries": [
            {
                "buffer": delivery["buffer"],
                "partId": delivery["partId"],
                "article": delivery["article"],
                "deliverQuantity": delivery["deliverQuantity"],
            }
            for delivery in deliveries
            if delivery["handlingMethod"] == "robotarm"
        ],
    }


def start_transport_process(active_batch_file):
    if not os.path.exists(TRANSPORT_SCRIPT):
        message = f"[TRANSPORT] transport.py niet gevonden: {TRANSPORT_SCRIPT}"
        print(message)
        append_transport_event(message)
        return None
    try:
        process = subprocess.Popen(
            [sys.executable, "-u", TRANSPORT_SCRIPT, active_batch_file],
            cwd=BASE_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except OSError as error:
        message = f"[TRANSPORT] Kon transport.py niet starten: {error}"
        print(message)
        append_transport_event(message)
        return None
    message = f"[TRANSPORT] transport.py gestart met PID {process.pid}"
    print(message)
    append_transport_event(message)
    threading.Thread(target=relay_transport_output, args=(process,), daemon=True).start()
    return process.pid


def start_main_control_process():
    """Launch main_control.py (robot sequence controller) as a background process."""
    if not os.path.exists(MAIN_CONTROL_SCRIPT):
        message = f"[MAIN_CONTROL] main_control.py niet gevonden: {MAIN_CONTROL_SCRIPT}"
        print(message)
        append_transport_event(message)
        return None
    try:
        process = subprocess.Popen(
            [sys.executable, "-u", MAIN_CONTROL_SCRIPT],
            cwd=BASE_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except OSError as error:
        message = f"[MAIN_CONTROL] Kon main_control.py niet starten: {error}"
        print(message)
        append_transport_event(message)
        return None
    message = f"[MAIN_CONTROL] main_control.py gestart met PID {process.pid}"
    print(message)
    append_transport_event(message)
    threading.Thread(target=relay_subprocess_output, args=(process, "[MAIN_CONTROL]"), daemon=True).start()
    return process.pid


def clear_transport_status():
    if os.path.exists(TRANSPORT_STATUS_FILE):
        try:
            os.remove(TRANSPORT_STATUS_FILE)
        except OSError:
            pass


def append_transport_event(line):
    text = (line or "").strip()
    if not text:
        return
    with TRANSPORT_STATUS_LOCK:
        status = read_json(TRANSPORT_STATUS_FILE, {"status": "idle", "commands": [], "events": []})
        events = status.get("events") or []
        events.append({
            "time": datetime.now().isoformat(timespec="seconds"),
            "line": text,
            "level": "error" if (" fail" in text.lower() or "aborted" in text.lower() or "storing" in text.lower()) else "info",
        })
        status["events"] = events[-400:]
        status["updatedAt"] = datetime.now().isoformat(timespec="seconds")
        write_json(TRANSPORT_STATUS_FILE, status)


@app.route("/api/transport/note", methods=["POST"])
def save_transport_note():
    data = request.get_json() or {}
    note = str(data.get("note") or "")
    status = read_json(TRANSPORT_STATUS_FILE, {"status": "idle", "commands": [], "events": []})
    status["operatorNote"] = note
    status["updatedAt"] = datetime.now().isoformat(timespec="seconds")
    write_json(TRANSPORT_STATUS_FILE, status)
    return jsonify({"operatorNote": note})


def relay_transport_output(process):
    if not process.stdout:
        return
    for line in process.stdout:
        text = line.rstrip()
        if not text:
            continue
        print(text)
        append_transport_event(text)
    process.wait()


def relay_subprocess_output(process, prefix=""):
    """Generic relay used for main_control.py output."""
    if not process.stdout:
        return
    for line in process.stdout:
        text = line.rstrip()
        if not text:
            continue
        print(f"{prefix} {text}" if prefix else text)
        append_transport_event(f"{prefix} {text}" if prefix else text)
    process.wait()


@app.route("/")
def index():
    return render_template(
        "index.html",
        slot_rows=SLOT_ROWS,
        slot_columns=SLOT_COLUMNS,
        slots=SLOTS,
    )


@app.route("/api/config")
def get_config():
    return jsonify(
        {
            "slotRows": SLOT_ROWS,
            "slotColumns": SLOT_COLUMNS,
            "slots": SLOTS,
            "hopperLayout": HOPPER_LAYOUT,
            "hopperTypeOptions": HOPPER_TYPE_OPTIONS,
            "startupNotes": {
                "totalHoppers": 25,
                "largePartsHandledBy": "robotarm",
            },
        }
    )


@app.route("/api/parts", methods=["GET"])
def get_parts():
    return jsonify(load_parts())


@app.route("/api/parts", methods=["POST"])
def create_part():
    data = request.get_json() or {}

    parts = load_parts()
    payload, error = validate_part_payload(data, parts)
    if error:
        return jsonify({"error": error}), 400

    next_id = max((part["id"] for part in parts), default=0) + 1
    part = normalize_part({"id": next_id, **payload})
    parts.append(part)
    save_parts(parts)
    return jsonify(part), 201


@app.route("/api/parts/<int:part_id>", methods=["PUT"])
def update_part(part_id):
    data = request.get_json() or {}
    parts = load_parts()
    existing_part = next((part for part in parts if part["id"] == part_id), None)
    if not existing_part:
        return jsonify({"error": "Onderdeel niet gevonden."}), 404

    payload, error = validate_part_payload(data, parts, current_part_id=part_id)
    if error:
        return jsonify({"error": error}), 400

    updated_part = normalize_part({"id": part_id, **payload})
    updated_parts = [updated_part if part["id"] == part_id else part for part in parts]
    save_parts(updated_parts)
    sync_part_details_in_batches(updated_part)
    return jsonify(updated_part)


@app.route("/api/parts/<int:part_id>", methods=["DELETE"])
def delete_part(part_id):
    parts = load_parts()
    updated_parts = [part for part in parts if part["id"] != part_id]
    if len(updated_parts) == len(parts):
        return jsonify({"error": "Onderdeel niet gevonden."}), 404
    save_parts(updated_parts)
    return jsonify({"status": "deleted"})


@app.route("/api/import/csv", methods=["POST"])
def preview_csv_import():
    file_storage = request.files.get("file")
    if not file_storage:
        return jsonify({"error": "Kies eerst een CSV-bestand."}), 400
    parsed_sets, error = parse_installation_sets_csv(file_storage)
    if error:
        return jsonify({"error": error}), 400
    return jsonify({"sets": parsed_sets})


@app.route("/api/import/csv/apply", methods=["POST"])
def apply_csv_import():
    data = request.get_json() or {}
    batch_size = int(data.get("batchSize") or 0)
    selected_set = data.get("set") or {}
    import_parts = selected_set.get("parts") or []

    if batch_size < 1:
        return jsonify({"error": "Batchgrootte moet minimaal 1 zijn."}), 400
    if not import_parts:
        return jsonify({"error": "Kies eerst een set uit de CSV."}), 400

    slots, created_count, updated_count = build_import_slots(import_parts, batch_size)
    part_count = sum(len(items) for items in slots.values())
    total_qty = sum(part.get("quantity", 0) for items in slots.values() for part in items)
    return jsonify(
        {
            "name": selected_set.get("name") or selected_set.get("setNumber") or "CSV batch",
            "setNumber": selected_set.get("setNumber"),
            "batchSize": batch_size,
            "slots": slots,
            "partCount": part_count,
            "totalQty": total_qty,
            "createdParts": created_count,
            "updatedParts": updated_count,
        }
    )


@app.route("/api/transport/status")
def get_transport_status():
    with TRANSPORT_STATUS_LOCK:
        if not os.path.exists(TRANSPORT_STATUS_FILE):
            return jsonify({"status": "idle", "commands": []})
        return jsonify(read_json(TRANSPORT_STATUS_FILE, {"status": "idle", "commands": []}))


@app.route("/api/batches", methods=["GET"])
def get_batches():
    return jsonify(load_batches())


@app.route("/api/batches", methods=["POST"])
def create_batch():
    data = request.get_json() or {}
    name = (data.get("name") or "").strip()
    batch_size = int(data.get("batchSize") or 0)
    slots = normalize_slots(data.get("slots"))

    if not name:
        return jsonify({"error": "Batchnaam is verplicht."}), 400
    if batch_size < 1:
        return jsonify({"error": "Batchgrootte moet minimaal 1 zijn."}), 400

    batch = {
        "id": str(uuid.uuid4())[:8].upper(),
        "name": name,
        "created": datetime.now().strftime("%d-%m-%Y"),
        "batchSize": batch_size,
        "slots": slots,
        "partCount": sum(len(items) for items in slots.values()),
        "totalQty": sum(part.get("quantity", 0) for items in slots.values() for part in items),
    }

    batches = load_batches()
    batches.insert(0, batch)
    save_batches(batches)
    return jsonify(batch), 201


@app.route("/api/batches/<batch_id>", methods=["PUT"])
def update_batch(batch_id):
    data = request.get_json() or {}
    name = (data.get("name") or "").strip()
    batch_size = int(data.get("batchSize") or 0)
    slots = normalize_slots(data.get("slots"))

    if not name:
        return jsonify({"error": "Batchnaam is verplicht."}), 400
    if batch_size < 1:
        return jsonify({"error": "Batchgrootte moet minimaal 1 zijn."}), 400

    batches = load_batches()
    batch = next((item for item in batches if item["id"] == batch_id), None)
    if not batch:
        return jsonify({"error": "Batch niet gevonden."}), 404

    batch["name"] = name
    batch["batchSize"] = batch_size
    batch["slots"] = slots
    batch["partCount"] = sum(len(items) for items in slots.values())
    batch["totalQty"] = sum(part.get("quantity", 0) for items in slots.values() for part in items)

    save_batches(batches)
    return jsonify(batch)


@app.route("/api/batches/<batch_id>", methods=["DELETE"])
def delete_batch(batch_id):
    batches = load_batches()
    updated_batches = [batch for batch in batches if batch["id"] != batch_id]
    save_batches(updated_batches)
    return jsonify({"status": "deleted"})


@app.route("/api/pick/<batch_id>", methods=["POST"])
def pick(batch_id):
    batches = load_batches()
    batch = next((item for item in batches if item["id"] == batch_id), None)
    if not batch:
        return jsonify({"error": "Batch niet gevonden"}), 404
    start_payload = build_batch_start_payload(batch)
    write_json(ACTIVE_BATCH_FILE, start_payload)
    clear_transport_status()
    pick_message = f"[PICK] Batch {batch_id} gestart - {batch['name']}"
    print(pick_message)
    append_transport_event(pick_message)
    transport_pid = start_transport_process(ACTIVE_BATCH_FILE)
    main_control_pid = start_main_control_process()
    return jsonify(
        {
            "status": "started",
            "batch": batch,
            "activeBatch": start_payload,
            "outputFile": os.path.relpath(ACTIVE_BATCH_FILE, BASE_DIR),
            "transportPid": transport_pid,
            "mainControlPid": main_control_pid,
            "startupNotes": {
                "totalHoppers": 25,
                "smallBoltHoppers": 5,
                "mediumBoltHoppers": 5,
                "largeBoltHoppers": 5,
                "washerHoppers": 10,
                "largePartsHandledBy": "robotarm",
            },
        }
    )


ensure_data_files()


if __name__ == "__main__":
    print("Pick & Place Automotive - http://127.0.0.1:5000")
    #app.run(host="0.0.0.0", port=5000, debug=True)
    from waitress import serve
    serve(app, host="0.0.0.0", port=5000)
