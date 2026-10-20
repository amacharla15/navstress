from setuptools import find_packages, setup

package_name = "navstress_eval"

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
    maintainer="Akshith Macharla",
    maintainer_email="amacharla@csuchico.edu",
    description="eval_runner node: RunEpisode/RunBatch action servers, metrics and results files.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "eval_runner = navstress_eval.eval_runner:main",
            "run_batch = navstress_eval.batch_client:main",
        ]
    },
)
