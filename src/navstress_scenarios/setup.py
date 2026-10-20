from setuptools import find_packages, setup

package_name = "navstress_scenarios"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="amacharla15",
    maintainer_email="amacharla15@users.noreply.github.com",
    description="Scenario schema plus scenario_manager, noise_injector, obstacle_mover nodes.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "sample_goals = navstress_scenarios.goals:main",
            "make_baseline = navstress_scenarios.generate:main",
        ]
    },
)
