#!/usr/bin/env python3

import argparse
import sys
import time

import lcm


def main() -> int:
    parser = argparse.ArgumentParser(description="Print incoming LCM channel names and payload sizes.")
    parser.add_argument(
        "--url",
        default="udpm://224.0.0.7:5006?ttl=1",
        help="LCM URL to bind to (e.g. udpm://224.0.0.7:5006?ttl=1)",
    )
    parser.add_argument(
        "--pattern",
        default=".*",
        help="LCM subscription regex (default: .*)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Stop after N messages (0 = run forever)",
    )
    args = parser.parse_args()

    lc = lcm.LCM(args.url)

    count = 0
    last_print = 0.0

    def handler(channel: str, data: bytes) -> None:
        nonlocal count, last_print
        count += 1
        now = time.time()
        # Throttle prints if floods happen
        if now - last_print > 0.0:
            print(f"#{count} channel={channel} bytes={len(data)}")
            last_print = now

    lc.subscribe(args.pattern, handler)

    print(f"Listening LCM on url={args.url} pattern={args.pattern} ...")
    try:
        while True:
            lc.handle()
            if args.limit and count >= args.limit:
                return 0
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
