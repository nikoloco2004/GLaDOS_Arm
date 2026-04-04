import asyncio
import os

from .server import run_server


def main() -> None:
    # :: = IPv6 dual-stack on Linux (also accepts IPv4 when ipv6.bindv6only=0). Override with PI_RUNTIME_HOST=0.0.0.0 for IPv4-only.
    host = os.environ.get("PI_RUNTIME_HOST", "::")
    port = int(os.environ.get("PI_RUNTIME_PORT", "8765"))
    try:
        asyncio.run(run_server(host, port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
