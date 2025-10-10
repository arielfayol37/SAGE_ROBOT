
import requests
import random
import json

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

