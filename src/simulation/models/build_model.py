#!/usr/bin/env python3

from xml.dom.minidom import Document
from pathlib import Path

from xacro import process_file


this_dir = Path(__file__).parent
all_xacros = this_dir.rglob("*.*.xacro")

for xacro in all_xacros:
    try:
        document: Document = process_file(xacro)  # type: ignore
    except:
        print(f"Error while processing {xacro}")
        raise
    string = document.toprettyxml()
    target = xacro.parent / xacro.stem
    print(f"Processing: <{target}>")
    with open(target, "w") as target_file:
        target_file.write(string)
