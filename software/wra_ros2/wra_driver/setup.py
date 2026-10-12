from glob import glob
from setuptools import setup

package_name = "wra_driver"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="landen-finkel-engineering",
    description="WRA-1 driver, kinematics and tasks",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "driver_node = wra_driver.driver_node:main",
        "voice_node = wra_driver.voice_node:main",
        "fake_esp32 = wra_driver.fake_esp32:main",
    ]},
)
