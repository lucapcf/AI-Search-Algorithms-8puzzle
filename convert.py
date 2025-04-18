import os
import csv

instances_file = "test_instances/8puzzle_instances.txt"
csv_directory = "./results"
output_directory = "./converted_txt"
os.makedirs(output_directory, exist_ok=True)

with open(instances_file, "r") as f:
    instance_states = [line.strip() for line in f if line.strip()]

for filename in os.listdir(csv_directory):
    if filename.endswith(".csv"):
        csv_path = os.path.join(csv_directory, filename)
        txt_filename = os.path.splitext(filename)[0] + ".txt"
        txt_path = os.path.join(output_directory, txt_filename)

        with open(csv_path, newline='') as csvfile, open(txt_path, "w") as outfile:
            reader = csv.reader(csvfile)
            try:
                headers = next(reader)
            except StopIteration:
                print(f"[Warning] {filename} is empty. Skipping.")
                continue

            for idx, row in enumerate(reader):
                if idx < len(instance_states):
                    instance = instance_states[idx]
                    full_row = instance + " & " + " & ".join(row) + " \\\\"
                    outfile.write(full_row + "\n")
                else:
                    print(f"[Warning] More rows in {filename} than instances in .txt file.")

        print(f"Converted {filename} -> {txt_filename}")
