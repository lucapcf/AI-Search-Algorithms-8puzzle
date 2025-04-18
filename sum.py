import csv
import os

csv_directory = "./results"
output_file = "averages.txt"

with open(output_file, "w") as out:
    for filename in os.listdir(csv_directory):
        if filename.endswith(".csv"):
            filepath = os.path.join(csv_directory, filename)
            out.write(f"\nProcessing: {filename}\n")

            with open(filepath, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                sums = {}
                count = 0

                for row in reader:
                    count += 1
                    for key, value in row.items():
                        sums[key] = sums.get(key, 0) + float(value)

                if count == 0:
                    out.write("  (Empty file)\n")
                    continue

                out.write("  Column Averages:\n")
                for key, total in sums.items():
                    out.write(f"    {key}: {total / count:.6f}\n")

print(f"Averages saved to {output_file}")
