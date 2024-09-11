import omni.physxdemos as demo
from pathlib import Path


class SettingsTestDemo(demo.Base):
    category = "TestDemo"
    demo_base_usd_url = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/").joinpath("SettingsDemoTest.usda"))
