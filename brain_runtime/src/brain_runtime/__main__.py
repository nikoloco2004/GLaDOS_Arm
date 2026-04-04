import asyncio
import logging
import os

from .client import BrainClient


async def _run() -> None:
    url = os.environ.get("PI_WS_URL", "ws://127.0.0.1:8765")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    client = BrainClient(url)
    await client.run_forever()


def main() -> None:
    asyncio.run(_run())


if __name__ == "__main__":
    main()
