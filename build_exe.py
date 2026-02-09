"""
Сборка одного исполняемого файла EDFViewerMonitor.exe из sensor_monitor.py.
Запуск из корня проекта: python tools/python_monitor/build_exe.py
Или из папки tools/python_monitor: python build_exe.py

Требуется: pip install pyinstaller
Результат: dist/EDFViewerMonitor.exe (рядом с build_exe.py — в tools/python_monitor/dist/)
"""
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
MONITOR_SCRIPT = SCRIPT_DIR / "sensor_monitor.py"
DIST_DIR = SCRIPT_DIR / "dist"
EXE_NAME = "EDFViewerMonitor"


def main() -> None:
    if not MONITOR_SCRIPT.exists():
        print(f"Не найден {MONITOR_SCRIPT}")
        sys.exit(1)

    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--onefile",           # один .exe без папки
        "--windowed",         # без консоли (GUI только)
        "--name", EXE_NAME,
        "--distpath", str(DIST_DIR),
        "--workpath", str(SCRIPT_DIR / "build"),
        "--specpath", str(SCRIPT_DIR),
        "--clean",
        str(MONITOR_SCRIPT),
    ]

    print("Запуск:", " ".join(cmd))
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit(r.returncode)

    exe_path = DIST_DIR / f"{EXE_NAME}.exe"
    if exe_path.exists():
        print(f"Готово: {exe_path}")
        print("Размеры окна сохраняются в EDFViewerMonitor_geometry.json рядом с exe.")
    else:
        print("Ошибка: exe не создан.")
        sys.exit(1)


if __name__ == "__main__":
    main()
