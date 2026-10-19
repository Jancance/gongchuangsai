from pathlib import Path
import unittest

from smart_carrier.config import load_config_dir, validate_deployment_config


class ConfigTests(unittest.TestCase):
    def test_repository_config_is_safe_placeholder(self):
        root = Path(__file__).parents[1]
        config = load_config_dir(root / "config")
        errors = validate_deployment_config(config)
        self.assertTrue(any("camera main" in error for error in errors))
        self.assertTrue(any("calibration" in error for error in errors))
        self.assertEqual(config["hardware"]["uart"]["logic_voltage"], 3.3)


if __name__ == "__main__":
    unittest.main()
