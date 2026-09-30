Import("env")

from gzip import compress
from pathlib import Path

project_dir = Path(env["PROJECT_DIR"])
source_path = project_dir / "data" / "index.html"
generated_dir = project_dir / ".pio" / "generated"
header_path = generated_dir / "web_index.h"

html = source_path.read_bytes()
compressed = compress(html, compresslevel=9, mtime=0)
byte_rows = [
    ", ".join(f"0x{byte:02x}" for byte in compressed[offset:offset + 16])
    for offset in range(0, len(compressed), 16)
]
header = (
    "#pragma once\n\n"
    "#include <Arduino.h>\n\n"
    "static const uint8_t WEB_INDEX_HTML_GZ[] PROGMEM = {\n  "
    + ",\n  ".join(byte_rows)
    + "\n};\n"
    "static constexpr size_t WEB_INDEX_HTML_GZ_LEN = sizeof(WEB_INDEX_HTML_GZ);\n"
)

generated_dir.mkdir(parents=True, exist_ok=True)
if not header_path.exists() or header_path.read_text() != header:
    header_path.write_text(header)

env.Append(CPPPATH=[str(generated_dir)])
