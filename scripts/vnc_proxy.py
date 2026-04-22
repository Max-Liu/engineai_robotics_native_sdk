#!/usr/bin/env python3
"""
noVNC WebSocket proxy for macOS + Docker Desktop.

Architecture:
  Browser (WebSocket) -> this proxy (Mac) -> docker exec socat -> x11vnc in container

Requires: pip3 install websockets
Usage:    python3 scripts/vnc_proxy.py [port]
"""

import asyncio
import http
import pathlib
import subprocess
import sys
import urllib.parse

try:
    from websockets.legacy.server import serve as ws_serve
except ImportError:
    try:
        from websockets.server import serve as ws_serve
    except ImportError:
        print("[error] Missing dependency: pip3 install websockets")
        sys.exit(1)

CONTAINER   = "engineai_robotics_env"
VNC_PORT    = 5900
LISTEN_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 6080
NOVNC_CACHE = pathlib.Path("/tmp/novnc_web")

MIME_TYPES = {
    ".html": "text/html",
    ".js":   "application/javascript",
    ".css":  "text/css",
    ".png":  "image/png",
    ".ico":  "image/x-icon",
    ".wasm": "application/wasm",
    ".map":  "application/json",
    ".json": "application/json",
}


def extract_novnc_files():
    if (NOVNC_CACHE / "vnc.html").exists():
        return
    NOVNC_CACHE.mkdir(parents=True, exist_ok=True)
    print(f"[proxy] Extracting noVNC files from container to {NOVNC_CACHE} ...")
    result = subprocess.run(
        ["docker", "exec", CONTAINER, "find", "/usr/share/novnc", "-type", "f"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"[proxy] Warning: could not list noVNC files: {result.stderr.strip()}")
        return
    for src_path in result.stdout.splitlines():
        rel = src_path[len("/usr/share/novnc/"):]
        dst = NOVNC_CACHE / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        cp = subprocess.run(["docker", "exec", CONTAINER, "cat", src_path], capture_output=True)
        if cp.returncode == 0:
            dst.write_bytes(cp.stdout)
    print("[proxy] noVNC files extracted.")


async def http_handler(path, request_headers):
    """Legacy websockets process_request: (path, headers_obj) -> (status, [(k,v)], body) | None"""
    if request_headers.get("Upgrade", "").lower() == "websocket":
        return None

    if path in ("/", ""):
        path = "/vnc.html"

    filepath = NOVNC_CACHE / urllib.parse.urlparse(path).path.lstrip("/")
    if not filepath.exists() or not filepath.is_file():
        return http.HTTPStatus.NOT_FOUND, [], b"Not found\n"

    content_type = MIME_TYPES.get(filepath.suffix.lower(), "application/octet-stream")
    body = filepath.read_bytes()
    return http.HTTPStatus.OK, [
        ("Content-Type", content_type),
        ("Content-Length", str(len(body))),
        ("Cache-Control", "no-cache"),
    ], body


async def vnc_handler(websocket, path):
    proc = await asyncio.create_subprocess_exec(
        "docker", "exec", "-i", CONTAINER,
        "socat", "-", f"TCP:localhost:{VNC_PORT}",
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
    )

    async def ws_to_vnc():
        try:
            async for msg in websocket:
                data = msg if isinstance(msg, bytes) else msg.encode()
                proc.stdin.write(data)
                await proc.stdin.drain()
        except Exception:
            pass
        finally:
            try:
                proc.stdin.close()
            except Exception:
                pass

    async def vnc_to_ws():
        try:
            while True:
                data = await proc.stdout.read(32768)
                if not data:
                    break
                await websocket.send(data)
        except Exception:
            pass

    try:
        await asyncio.gather(ws_to_vnc(), vnc_to_ws())
    except Exception:
        pass
    finally:
        try:
            proc.terminate()
        except Exception:
            pass


async def main():
    extract_novnc_files()
    print(f"[proxy] Listening on http://localhost:{LISTEN_PORT}/vnc.html")
    print("[proxy] Press Ctrl+C to stop.")
    async with ws_serve(
        vnc_handler,
        "localhost",
        LISTEN_PORT,
        process_request=http_handler,
        subprotocols=["binary"],
        max_size=None,
        compression=None,
        ping_interval=None,
    ):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[proxy] Stopped.")
