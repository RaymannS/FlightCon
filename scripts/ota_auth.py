Import("env")

from pathlib import Path
import re


def _read_define(file_path: Path, define_name: str) -> str:
    pattern = re.compile(r'^\s*#define\s+' + re.escape(define_name) + r'\s+"([^"]*)"\s*$', re.MULTILINE)
    match = pattern.search(file_path.read_text())
    if not match:
        raise Exception(f"Missing {define_name} in {file_path}")
    return match.group(1)


def _read_int_define(file_path: Path, define_name: str) -> int:
    pattern = re.compile(r'^\s*#define\s+' + re.escape(define_name) + r'\s+(\d+)\s*$', re.MULTILINE)
    match = pattern.search(file_path.read_text())
    if not match:
        raise Exception(f"Missing {define_name} in {file_path}")
    return int(match.group(1))


project_dir = Path(env.subst("$PROJECT_DIR"))
credentials = project_dir / "include" / "wifi_credentials.h"
ota_password = _read_define(credentials, "WIFI_AP_PASSWORD")

env.Replace(
    UPLOAD_FLAGS=[
        "--port=3232",
        f"--auth={ota_password}",
    ]
)
