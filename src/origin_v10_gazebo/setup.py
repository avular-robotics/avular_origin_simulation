import os  # Operating system library
from glob import glob  # Handles file path names
from setuptools import setup  # Facilitates the building of packages

package_name = "origin_v10_gazebo"

# Path of the current directory
cur_directory_path = os.path.abspath(os.path.dirname(__file__))

# Get all the gazebo models recursively and keep the directory structure
model_src_paths = glob("models/**/*.*", recursive=True)
model_dest_paths = [
    os.path.join("share", package_name, os.path.dirname(path))
    for path in model_src_paths
]

data_files = list(zip(model_dest_paths, [[path] for path in model_src_paths]))

data_files.extend(
    [
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Path to the launch file
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Path to the world file
        (os.path.join("share", package_name, "worlds/"), glob("./worlds/*")),
        # (os.path.join('share', package_name,'models/'), glob('./models/**/*.*', recursive=True)),
        # Path to the config files
        (os.path.join("share", package_name, "config/"), glob("./config/*")),
    ]
)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Joris Sijs",
    maintainer_email="j.sijs@avular.com",
    description="Simulation environment for the Avular Origin V1.0",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_specifics = origin_v10_gazebo.origin_specifics:main",
        ],
    },
)
