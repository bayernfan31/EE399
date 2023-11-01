import csv
import numpy as numpy

def read_csv(file_path, value_type):
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)

        # Skip first two lines
        next(csv_reader)
        next(csv_reader)

        all_values = []
        row_num = 3
        for row in csv_reader:
            # Reading one row, skipping lines not neccessary as csv contains both angles and positions 
            if value_type == 'a' and row_num % 2 == 1:
                values = [float(cell.strip('[').strip(']')) for cell in row]
                all_values.append(values)
            elif value_type == 'c' and row_num % 2 == 0:
                values = [float(cell.strip('[').strip(']')) for cell in row]
                all_values.append(values)
            row_num+= 1
        return all_values 