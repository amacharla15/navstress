from setuptools import find_packages, setup

package_name = "navstress_fakesim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/world.yaml", "config/goals.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Akshith Macharla",
    maintainer_email="amacharla@csuchico.edu",
    description="CI-only fake simulator and fake NavigateToPose server.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fake_sim = navstress_fakesim.fake_sim:main",
        ]
    },
)
