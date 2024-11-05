from glob import glob
from setuptools import setup, find_packages

package_name = "radiolink"
package_share = "share/" + package_name

setup(
    name=package_name,
    version="0.1.0",
    maintainer="Igor Beschastnov",
    maintainer_email="beschastnovigor@gmail.com",
    description="Rover radiolink",
    license="MIT",
    # Const settings
    packages=find_packages(exclude="test"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (package_share, ["package.xml"]),
        (package_share + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{package_name} = {package_name}.__main__:main"],
    },
)
