from setuptools import setup


try:
  from catkin_pkg.python_setup import generate_distutils_setup
except ImportError:
  setup(
    name="create2remix",
    version="0.1",
    author="Shuhao Wu",
    author_email="shuhao@shuhaowu.com",
    description="Yet another Python library for the iRobot Create 2",
    license="GPLv3",
    packages=["create2remix"],
    install_requires=["pyserial==3.4", "six>=1.15"],
    classifiers=[
      "Development Status :: 3 - Alpha",
      "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
      "Programming Language :: Python :: 2.7",
      "Programming Language :: Python :: 3",
    ]
  )
else:
  d = generate_distutils_setup(
    packages=["create2remix"],
    package_dir={"": "."}
  )

  # catkin package setup
  setup(**d)
