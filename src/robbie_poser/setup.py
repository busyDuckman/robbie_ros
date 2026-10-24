from setuptools import setup

package_name = "robbie_poser"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="busyDuckman",
    maintainer_email="the.duckman@gmail.com",
    description="A small window for posing the neck by hand.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "poser = robbie_poser.poser:main",
        ],
    },
)
