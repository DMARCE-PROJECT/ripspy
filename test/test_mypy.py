from pathlib import Path

from ament_mypy.main import main

import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy():
    config_path = Path(__file__).parent / 'config' / 'mypy.ini'
    rc = main(argv=['--exclude', 'test', '--config', str(config_path.resolve())])
    assert rc == 0, 'Found code style errors / warnings'