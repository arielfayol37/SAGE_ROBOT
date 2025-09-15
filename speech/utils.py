
import requests
import random
import json

feasibility_url = 'http://127.0.0.1:8000/check_feasible_items/'  # URL to check feasibility
order_url = 'http://127.0.0.1:8000/order_items/'  # URL to place the order
def check_feasibility(order):
    payload = order
    return str(requests.post(feasibility_url, json={'items': payload}).json())

def place_order(order, tip=None):
    tip = round(random.uniform(0, 10)) if tip is None else tip
    payload = order
    return str(requests.post(order_url, json={'items': payload, 'tip':round(random.uniform(0, 10), 2)}).json())

# Function to read 'openai_key' from the JSON file
def read_openai_key(json_file):
    try:
        # Open the JSON file
        with open(json_file, 'r') as file:
            # Load the JSON data
            data = json.load(file)
            # Access the 'openai_key' value
            openai_key = data.get("openai_key")
            return openai_key
    except FileNotFoundError:
        print(f"Error: {json_file} not found.")
    except json.JSONDecodeError:
        print(f"Error: {json_file} contains invalid JSON.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def read_system_prompts(json_file):
    with open(json_file, 'r') as file:
        data = json.load(file)
        return data["system_prompt_0"], data["system_prompt_1"]

