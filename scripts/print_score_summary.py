#!/usr/bin/env python3
import pathlib
import sys

import yaml


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: print_score_summary.py <scoring.yaml>", file=sys.stderr)
        return 2

    scoring_path = pathlib.Path(sys.argv[1])
    if not scoring_path.is_file():
        print(f"missing scoring file: {scoring_path}", file=sys.stderr)
        return 1

    with scoring_path.open("r", encoding="utf-8") as scoring_file:
        data = yaml.safe_load(scoring_file)

    print(f"file: {scoring_path}")

    if isinstance(data, dict):
        for key in ("total_score", "score", "phase", "status"):
            if key in data:
                print(f"{key}: {data[key]}")

        trials = data.get("trials")
        if isinstance(trials, list):
            for index, trial in enumerate(trials, start=1):
                if not isinstance(trial, dict):
                    continue
                label = trial.get("name") or trial.get("trial") or f"trial_{index}"
                score = trial.get("score", "n/a")
                success = trial.get("success", "n/a")
                print(f"{label}: score={score} success={success}")
    else:
        print(data)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
