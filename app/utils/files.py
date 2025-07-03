import os

def calculate_total_size(bag_path: str) -> int:
    total_size = 0
    for root, _, files in os.walk(bag_path):
        for file in files:
            full_path = os.path.join(root, file)
            total_size += os.path.getsize(full_path)
    return total_size

