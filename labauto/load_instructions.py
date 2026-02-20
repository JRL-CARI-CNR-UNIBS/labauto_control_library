import ast

# Read the file and parse instructions
def loadInstructions(filename):
    instructions = []
    with open(filename, "r") as file:
        for line in file:
            line = line.strip()  # Remove leading/trailing whitespace
            if not(line):
                continue
            if ": [" in line:  # If the line contains a list
                key, value = line.split(": ", 1)
                value = ast.literal_eval(value)  # Convert string "[x, y]" to a list
            else:
                key, value = line.split(": ")
                value = float(value)  # Convert to number
            instructions.append(f"{key}: {value}")

    return instructions


import re

def strip_comments(line: str) -> str:
    # remove ; comments
    line = line.split(";", 1)[0]
    # remove ( ... ) comments
    line = re.sub(r"\([^)]*\)", "", line)
    return line.strip()

def parse_words(line: str):
    # returns dict like {"G": 1, "X": 10.0, "Y": -2.5}
    words = {}
    for m in re.finditer(r"([A-Za-z])\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", line):
        letter = m.group(1).upper()
        num = float(m.group(2))
        # If multiple same letters appear, last wins (common behavior)
        words[letter] = num
    return words

def loadGCode(
    filename: str,
    *,
    gcode_units="mm",     # "mm" or "m" (what the gcode numbers mean)
    sim_units="m",        # "m" expected by your simulator
    start_pos=(0.0, 0.0, 0.0)
):
    # scale factor from gcode units to simulator units
    unit_scale = 1.0
    if gcode_units == "mm" and sim_units == "m":
        unit_scale = 0.001
    elif gcode_units == "m" and sim_units == "mm":
        unit_scale = 1000.0

    x, y, z = start_pos
    absolute = True  # default to G90-ish

    out = []

    with open(filename, "r", encoding="utf-8") as f:
        for raw in f:
            line = strip_comments(raw)
            if not line:
                continue

            words = parse_words(line)
            if not words:
                continue

            # mode switches
            if words.get("G") == 90:
                absolute = True
                continue
            if words.get("G") == 91:
                absolute = False
                continue

            g = words.get("G")

            # dwell
            if g == 4:
                if "S" in words:
                    seconds = float(words["S"])
                elif "P" in words:
                    seconds = float(words["P"]) / 1000.0
                else:
                    seconds = 0.0
                out.append(f"pause: {seconds}")
                continue

            # motion
            if g in (0, 1):
                nx, ny, nz = x, y, z
                if "X" in words:
                    nx = (words["X"] * unit_scale) if absolute else (x + words["X"] * unit_scale)
                if "Y" in words:
                    ny = (words["Y"] * unit_scale) if absolute else (y + words["Y"] * unit_scale)
                if "Z" in words:
                    nz = (words["Z"] * unit_scale) if absolute else (z + words["Z"] * unit_scale)

                x, y, z = nx, ny, nz
                out.append(f"move: [{x}, {y}, {z}]")
                continue

            # ignore everything else (M codes, F, etc.)

    return out