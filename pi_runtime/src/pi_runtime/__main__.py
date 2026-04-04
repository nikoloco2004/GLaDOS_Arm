import asyncio
import os

from .server import run_server


def main() -> None:
    host = os.environ.get("PI_RUNTIME_HOST", "0.0.0.0")
    port = int(os.environ.get("PI_RUNTIME_PORT", "8765"))
    asyncio.run(run_server(host, port))


if __name__ == "__main__":
    main()
