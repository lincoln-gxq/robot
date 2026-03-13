from setuptools import find_packages, setup

package_name = "path_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lincoln-gxq",
    maintainer_email="lincoln-gxq@example.com",
    description="Path tracking controller for sweeping robot simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_tracker_node = path_tracker.path_tracker_node:main",
        ],
    },
)
