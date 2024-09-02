from glob import glob
from setuptools import setup, find_packages

package_name = "simulation"
package_share = "share/" + package_name

setup(
    name=package_name,
    version="0.1.0",
    maintainer="Igor Beschastnov",
    maintainer_email="beschastnovigor@gmail.com",
    description="Simulations and visualizations for Russian Rover Challenge",
    license="MIT",
    # Const settings
    packages=find_packages(exclude="test"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (package_share, ["package.xml"]),
        (package_share + '/launch', glob('launch/*.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"]
)
