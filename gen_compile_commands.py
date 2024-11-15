import json
import re
from pathlib import Path
import shutil
import os

def parse_makefile(makefile_content):
    """
    Parses the Makefile content to extract compiler arguments, include paths, and source files.
    """
    compiler_path = shutil.which("arm-none-eabi-gcc")
    if not compiler_path:
        raise FileNotFoundError("arm-none-eabi-gcc not found in the system PATH.")
    else:
        compiler_path = compiler_path.replace("\\", "/")
        
    print(compiler_path)
    
    parsed_data = {
        "compiler": compiler_path,
        "c_flags": [],
        "include_dirs": [],
        "source_files": [],
        "output_dir": "build"
    }
    
    # Regex to match inline var definitions (ex. BOARD_BASE = typec)
    inline_var_pattern = re.compile(r"(\w+)\s*=\s*([^\n\\]+)\n")

    dynamic_vars = {}
    for match in inline_var_pattern.finditer(makefile_content):
        var_name = match.group(1).strip()
        var_value = match.group(2).strip()
        dynamic_vars[var_name] = var_value
        
    for i in dynamic_vars:
        print(i," = ", dynamic_vars[i])
        dynamic_vars[i] = expand_variables(dynamic_vars[i], dynamic_vars)
        
    # dynamic_vars = {
    #     "BOARD_BASE": "control-base/typec-board-base",
    #     "CONTROL_BASE": "control-base",
    #     "BUILD_DIR": "build"
    # }

    # Regex to match C_SOURCES and C_INCLUDES blocks in the Makefile
    c_sources_pattern = re.compile(r"C_SOURCES\s*=\s*(.*?)(?=\n\n|$)", re.S)
    c_includes_pattern = re.compile(r"C_INCLUDES\s*=\s*(.*?)(?=\n\n|$)", re.S)
    
    # Extract and expand C_SOURCES
    if c_sources_match := c_sources_pattern.search(makefile_content):
        sources = c_sources_match.group(1).replace("\\", "").split()
        parsed_data["source_files"] = [expand_variables(src, dynamic_vars) for src in sources]

    # Extract and expand C_INCLUDES
    if c_includes_match := c_includes_pattern.search(makefile_content):
        includes = c_includes_match.group(1).replace("\\", "").split()
        parsed_data["include_dirs"] = [expand_variables(inc, dynamic_vars) for inc in includes]

    return parsed_data, dynamic_vars


# Expanding variables in paths
def expand_variables(text, variables):
    """
    Expands variables in the given text using the provided dictionary.
    """
    for var, value in variables.items():
        text = text.replace(f"$({var})", value)
        text = text.replace(f"${{{var}}}", value)
    return text


# Generate compile_commands.json entries
def generate_compile_commands(parsed_data, project_dir):
    """
    Generates a single compile command entry for `compile_commands.json` format.
    Only the first source file in `parsed_data` is used to avoid duplication.
    """
    if not parsed_data["source_files"]:
        raise ValueError("No source files found to generate compile command.")
    
    # Select only the first source file (main.c)
    src_file = parsed_data["source_files"][0]
    
    # Define the full path for the selected source file
    full_src_path = Path(project_dir) / src_file
    full_output_path = Path(project_dir) / parsed_data["output_dir"] / Path(src_file).with_suffix(".o")

    full_src_path = str(full_src_path).replace("\\", "/")
    full_output_path = str(full_output_path).replace("\\", "/")

    # Build the command structure
    command_entry = {
        "arguments": [
            parsed_data["compiler"],
            "-c",
            *parsed_data["c_flags"],
            *parsed_data["include_dirs"],
            "-o", full_output_path,
            full_src_path
        ],
        "directory": project_dir,
        "file": full_src_path,
        "output": full_output_path
    }
    
    return [command_entry]

def main():
    project_dir = os.getcwd().replace("\\", "/")
    
    # dynamic_vars = {
    #     "BOARD_BASE": "control-base/typec-board-base",
    #     "CONTROL_BASE": "control-base",
    #     "BUILD_DIR": "build"
    # }

    makefile_path = './Makefile'
    with open(makefile_path, 'r') as file:
        makefile_content = file.read()

    parsed_data, dynamic_vars = parse_makefile(makefile_content)
    compile_commands = generate_compile_commands(parsed_data, project_dir)

    with open('compile_commands.json', 'w') as f:
        json.dump(compile_commands, f, indent=2)
        
    print("Successfully generated compile_commands.json file.")

if __name__ == "__main__":
    main()
