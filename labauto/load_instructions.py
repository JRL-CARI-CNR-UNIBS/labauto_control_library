import ast

# Read the file and parse instructions
def loadInstructions(filename):
    instructions = []
    with open(filename, "r") as file:
        for line in file:
            line = line.strip()  # Remove leading/trailing whitespace
            if ": [" in line:  # If the line contains a list
                key, value = line.split(": ", 1)
                value = ast.literal_eval(value)  # Convert string "[x, y]" to a list
            else:
                key, value = line.split(": ")
                value = float(value)  # Convert to number
            instructions.append(f"{key}: {value}")

    return instructions
