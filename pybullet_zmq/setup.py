import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pybullet_zmq",
    version="0.0",
    maintainer="Dominic Reber",
    maintainer_emal="dominic@aica.tech",
    description="This package implements a ZMQ interface for the PyBullet simulation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # url="https://github.com/pypa/sampleproject",
    packages=setuptools.find_packages(),
    install_requires=[
        "pybullet_simulation",
        "pyyaml",
        "zmq",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Operating System :: Unix",
    ],
    python_requires='>=3.8',
    scripts=["bin/zmq-simulator"],
)