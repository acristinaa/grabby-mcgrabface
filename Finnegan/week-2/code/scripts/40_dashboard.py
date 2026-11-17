"""Startet den Web-Steuerstand.

    .venv/bin/python scripts/40_dashboard.py
    -> http://127.0.0.1:8000

Optionen: --host --port --reload
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


def main():
    ap = argparse.ArgumentParser(description="G1 Steuerstand")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--reload", action="store_true", help="Neuladen bei Codeaenderung")
    args = ap.parse_args()

    import uvicorn
    print(f"Steuerstand:  http://{args.host}:{args.port}")
    uvicorn.run("dashboard.server:app", host=args.host, port=args.port,
                reload=args.reload, log_level="warning")


if __name__ == "__main__":
    main()
