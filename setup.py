from setuptools import setup
import os

data_files = []
packages = ["create2remix"]
entry_points = {"console_scripts": []}
install_requires = ["pyserial==3.4", "six>=1.14"]

if os.environ.get("ROS_DISTRO"): # kind of a hack, but should work
  data_files.append(("share/ament_index/resource_index/packages", ["resource/create2remix"])),
  data_files.append(("share/create2remix", ["package.xml"])),
  packages.append("create2remix.ros2")
  entry_points["console_scripts"].append("create2remix = create2remix.ros2.node:main")
  install_requires.append("transforms3d>=0.3.1")

setup(
  name="create2remix",
  version="0.2",
  author="Shuhao Wu",
  author_email="shuhao@shuhaowu.com",
  data_files=data_files,
  description="Yet another Python library for the iRobot Create 2",
  license="GPLv3",
  packages=packages,
  entry_points=entry_points,
  install_requires=install_requires,
  classifiers=[
    "Development Status :: 3 - Alpha",
    "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
    "Programming Language :: Python :: 2.7",
    "Programming Language :: Python :: 3",
  ],
)