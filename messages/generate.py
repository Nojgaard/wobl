#!/usr/bin/env python3
import subprocess
from pathlib import Path

messages_dir = Path(__file__).parent
proto_files = list(messages_dir.glob("*.proto"))

for proto_file in proto_files:
    print(f"Generating {proto_file.name}...")
    Path("woblpy/messages").mkdir(parents=True, exist_ok=True)
    Path("woblcpp/include/wobl/msg").mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "protoc",
            "--python_out=woblpy/messages",
            "--pyi_out=woblpy/messages",
            "--cpp_out=woblcpp/include/wobl/msg",
            "--proto_path=messages",
            proto_file.name,
        ],
        check=True,
    )

print("Done!")
