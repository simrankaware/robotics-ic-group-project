import numpy as np
import csv

def read_csv(filename):
    x_vals, y_vals = [], []
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        for row in reader:
            if row[0].strip().lower() == "x" and row[1].strip().lower() == "y":
                continue  # Ensure first line "x, y" is excluded
            x, y = map(float, row)
            x_vals.append(x)
            y_vals.append(y)
    return np.array(x_vals), np.array(y_vals)

def compute_covariance_matrix(x, y):
    N = len(x)
    x_mean, y_mean = np.mean(x), np.mean(y)
    
    cov_xx = np.sum((x - x_mean) ** 2) / N
    cov_yy = np.sum((y - y_mean) ** 2) / N
    cov_xy = np.sum((x - x_mean) * (y - y_mean)) / N
    
    return np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])

if __name__ == "__main__":
    filename = "coords.csv"  # Update this with your actual filename
    x, y = read_csv(filename)
    covariance_matrix = compute_covariance_matrix(x, y)
    print("Covariance Matrix:")
    print(covariance_matrix)
