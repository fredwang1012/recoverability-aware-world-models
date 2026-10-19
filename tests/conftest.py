import os
import sys

MINING = os.path.join(os.path.dirname(__file__), "..", "mining")
sys.path.insert(0, MINING)


def pytest_configure(config):
    # scipy's Sobol warns when a batch size is not a power of 2 (algos.Sobol / GPSearch ask 4 or odd sizes); harmless for QMC use
    config.addinivalue_line("filterwarnings", "ignore:The balance properties of Sobol:UserWarning")
