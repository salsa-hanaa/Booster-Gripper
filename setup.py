from setuptools import setup, find_packages

package_name = "booster_gripper"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    install_requires=[
        "setuptools",
        "pyserial",
    ],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your@email.com",
    description="Booster gripper package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gripper_node = booster_gripper.gripper_main:main",
        ],
    },
)