#!/usr/bin/env python3
import csv
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: evaluate_grasp.py <csv_file>")
        return

    csv_file = sys.argv[1]
    rows = []

    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    rows_sorted = sorted(rows, key=lambda x: float(x["score"]), reverse=True)

    print("Top 5 grasp conditions:")
    for row in rows_sorted[:5]:
        print(
            f'exp_id={row["exp_id"]}, '
            f'z={row["grasp_z"]}, yaw={row["yaw_deg"]}, force={row["grip_force"]}, '
            f'score={row["score"]}'
        )

if __name__ == "__main__":
    main()