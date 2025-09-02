#!/usr/bin/env python3

import sys
import yaml


def infer_cpp_type(value):
    if isinstance(value, bool):
        return "bool"
    elif isinstance(value, int):
        return "int"
    elif isinstance(value, float):
        return "double"
    elif isinstance(value, str):
        return "std::string"
    elif isinstance(value, list):
        if all(isinstance(x, float) for x in value):
            return "std::vector<double>"
        elif all(isinstance(x, int) for x in value):
            return "std::vector<int>"
        elif all(isinstance(x, str) for x in value):
            return "std::vector<std::string>"
        else:
            print("ERROR: Unsupported list element type")
            exit(1)
    else:
        print("ERROR: Unsupported type")
        exit(1)


def parse_yaml_to_struct(yaml_path, struct_name="RosParams"):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    # 兼容ros__parameters嵌套
    if isinstance(data, dict) and len(data) == 1:
        data = list(data.values())[0].get(
            "ros__parameters", data[list(data.keys())[0]])
    struct_lines = [f"struct {struct_name} " + "{"]
    for key, value in data.items():
        cpp_type = infer_cpp_type(value)
        struct_lines.append(f"  {cpp_type} {key};")
    struct_lines.append("};")
    return "\n".join(struct_lines)


if __name__ == "__main__":
    argc, argv = len(sys.argv), sys.argv
    yaml_path = None
    if argc == 1:
        yaml_path = "settings.yaml"
    elif argc == 2:
        yaml_path = argv[1]
    else:
        print("ERROR: Invalid arguments")
        exit(1)
    struct_code = parse_yaml_to_struct(yaml_path)
    print(struct_code)
