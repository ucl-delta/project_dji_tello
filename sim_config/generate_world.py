import json
import argparse

def generate_drones_json(drone_names, output_file, world_base = None):

    # Base template for the JSON structure
    base_json = {
        "world_name": "empty",
        "drones": []
    }
    if world_base:
        with open(world_base, 'r') as f:
            base_json = json.load(f)
        if "drones" not in world_base:
            base_json["drones"] = []

    # Split the comma-separated drone names
    drone_names_list = drone_names.split(',')

    # Populate the drones list in the base JSON
    for idx, drone_name in enumerate(drone_names_list):
        drone_data = {
            "model_type": "quadrotor_base",
            "model_name": drone_name.strip(),
            "xyz": [
                float(idx),  # X coordinate: idx for spacing them in a line
                0.0,         # Y coordinate
                0.3          # Z coordinate
            ],
            "rpy": [
                0,
                0,
                0
            ],
            # "payload": [
            #     {
            #         "model_name": "hd_camera",
            #         "model_type": "hd_camera"
            #     }
            # ],
            "flight_time": 60
        }
        base_json["drones"].append(drone_data)

    # Write the JSON to the specified file
    with open(output_file, 'w') as f:
        json.dump(base_json, f, indent=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate drones JSON file.')
    parser.add_argument('--drones', required=True, help='Comma-separated list of drone names.')
    parser.add_argument('--output', required=True, help='Output file to save the JSON.')
    parser.add_argument("--world_base", help="World Base JSON to add drones to")

    args = parser.parse_args()

    generate_drones_json(args.drones, args.output, world_base=args.world_base)

    print(f"Generated JSON saved to {args.output}")
