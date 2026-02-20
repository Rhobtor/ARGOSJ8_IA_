#!/usr/bin/env python3
"""Minimal TCP port checker.

Usage:
  python3 scripts/check_tcp_port.py 192.168.1.17 21000
  python3 scripts/check_tcp_port.py 192.168.1.17 21000 --timeout 1.0
  python3 scripts/check_tcp_port.py 192.168.1.17 21000 --send 'PING\n' --recv 64
    python3 scripts/check_tcp_port.py 192.168.1.17 21000 --duration 2
    python3 scripts/check_tcp_port.py 192.168.1.17 21000 --duration 2 --display hex
    python3 scripts/check_tcp_port.py 192.168.1.17 21000 --duration 5 --out /tmp/fixposition_21000.bin

Notes:
- `ping` only tells you the host answers ICMP; it does NOT mean a TCP port is open.
- "Listening" for TCP generally means your connect() succeeds.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time


def _hexdump(data: bytes, width: int = 16) -> str:
    lines: list[str] = []
    for i in range(0, len(data), width):
        chunk = data[i : i + width]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{i:08x}  {hex_part:<{width*3}}  |{ascii_part}|")
    return "\n".join(lines)


def _looks_texty(data: bytes) -> bool:
    if not data:
        return True
    # Heuristic: treat as text if it contains mostly printable chars/newlines/tabs.
    printable = 0
    for b in data[:256]:
        if b in (9, 10, 13) or 32 <= b < 127:
            printable += 1
    return (printable / min(len(data), 256)) > 0.85


def main() -> int:
    parser = argparse.ArgumentParser(description="Check if a TCP host:port is reachable (connect succeeds).")
    parser.add_argument("host", help="IP or hostname")
    parser.add_argument("port", type=int, help="TCP port")
    parser.add_argument("--timeout", type=float, default=1.0, help="Seconds (default: 1.0)")
    parser.add_argument("--send", default=None, help="Optional data to send after connect")
    parser.add_argument("--recv", type=int, default=0, help="Bytes to recv after send (default: 0)")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to keep reading and printing incoming data after connect (default: 0 = don't read)",
    )
    parser.add_argument(
        "--display",
        choices=("auto", "utf8", "hex"),
        default="auto",
        help="How to display received bytes when --duration > 0 (default: auto)",
    )
    parser.add_argument(
        "--chunk",
        type=int,
        default=4096,
        help="Bytes per recv() while streaming (default: 4096)",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Optional path to write raw received bytes while streaming (e.g. /tmp/stream.bin)",
    )
    args = parser.parse_args()

    if not (1 <= args.port <= 65535):
        print(f"Invalid port: {args.port}", file=sys.stderr)
        return 2

    t0 = time.time()
    try:
        with socket.create_connection((args.host, args.port), timeout=args.timeout) as s:
            s.settimeout(args.timeout)
            dt_ms = (time.time() - t0) * 1000.0
            local_ip, local_port = s.getsockname()[:2]
            print(f"OK: connected to {args.host}:{args.port} in {dt_ms:.1f} ms (local {local_ip}:{local_port})")

            if args.send is not None:
                data = args.send.encode("utf-8", errors="replace")
                s.sendall(data)
                print(f"Sent {len(data)} bytes")

                if args.recv > 0:
                    resp = s.recv(args.recv)
                    try:
                        printable = resp.decode("utf-8", errors="replace")
                    except Exception:
                        printable = repr(resp)
                    print(f"Received {len(resp)} bytes: {printable!r}")

            if args.duration and args.duration > 0:
                print(f"\n--- streaming for {args.duration:.3f}s (display={args.display}) ---")
                out_fh = open(args.out, "wb") if args.out else None
                end_t = time.time() + args.duration
                total = 0
                try:
                    while time.time() < end_t:
                        try:
                            chunk = s.recv(max(1, args.chunk))
                        except socket.timeout:
                            continue

                        if not chunk:
                            print("[peer closed connection]")
                            break

                        total += len(chunk)
                        if out_fh:
                            out_fh.write(chunk)

                        mode = args.display
                        if mode == "auto":
                            mode = "utf8" if _looks_texty(chunk) else "hex"

                        if mode == "utf8":
                            text = chunk.decode("utf-8", errors="replace")
                            # Print raw as-is (may include newlines already)
                            sys.stdout.write(text)
                            sys.stdout.flush()
                        else:
                            print(_hexdump(chunk))
                finally:
                    if out_fh:
                        out_fh.close()

                print(f"\n--- done (total received: {total} bytes) ---")
                if args.out:
                    print(f"Wrote raw stream to: {args.out}")

            return 0

    except ConnectionRefusedError:
        dt_ms = (time.time() - t0) * 1000.0
        print(f"NO: connection refused to {args.host}:{args.port} ({dt_ms:.1f} ms).")
        print("    The host is reachable, but nothing is listening on that TCP port (or a firewall actively refused).")
        return 1

    except socket.timeout:
        dt_ms = (time.time() - t0) * 1000.0
        print(f"NO: timeout connecting to {args.host}:{args.port} after {dt_ms:.1f} ms.")
        print("    Possible causes: port closed (dropped), firewall, wrong IP/port, device not serving that port.")
        return 1

    except OSError as e:
        dt_ms = (time.time() - t0) * 1000.0
        print(f"NO: OS error connecting to {args.host}:{args.port} after {dt_ms:.1f} ms: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
