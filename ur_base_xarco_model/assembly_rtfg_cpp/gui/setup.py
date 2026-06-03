from setuptools import find_packages, setup

package_name = "rtfg_gui"

setup(
    name=package_name,
    version="0.2.0",
    description="PyQt5-based desktop control panel for assembly_rtfg_cpp",
    maintainer="liuxiaopeng",
    maintainer_email="dev@example.com",
    license="Apache-2.0",
    packages=find_packages(),
    package_data={
        package_name: [],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/rtfg_display.rviz"]),
        ("lib/" + package_name, ["scripts/rtfg_launcher.py"]),
    ],
    install_requires=[
        "PyQt5>=5.15",
    ],
    zip_safe=False,
)
