"""Utilities for scenario JSON paths."""
import os


def _get_json_dir():
    d = os.environ.get("JSON_DIR", os.path.join(os.getcwd(), "json"))
    os.makedirs(d, exist_ok=True)
    return d


def get_do_scenario_path():
    return os.path.join(_get_json_dir(), "do_scenario.json")


def get_pick_place_config_path():
    return os.path.join(_get_json_dir(), "pick_place_config.json")


def get_pick_place_times_path():
    return os.path.join(_get_json_dir(), "pick_place_times.json")

