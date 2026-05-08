"""Shared pytest fixtures and command-line options."""

import pytest


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--port",
        default=None,
        help="Serial port for hardware tests (auto-detected if omitted)",
    )


@pytest.fixture(scope="session")
def serial_port(request: pytest.FixtureRequest) -> str | None:
    return request.config.getoption("--port")
