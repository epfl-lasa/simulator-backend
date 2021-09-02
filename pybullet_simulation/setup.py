import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pybullet_simulation",
    version="0.0",
    description="This package implements the PyBullet simulation",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # url="https://github.com/pypa/sampleproject",
    packages=setuptools.find_packages(),
    install_requires=[
        "control-libraries==3.1.0",
        "numpy==1.20.2",
        "numpy-quaternion==2021.4.5.14.42.35",
        "numba==0.53.1",
        "scipy==1.6.2",
        "pybullet==3.1.7"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Operating System :: Unix",
    ],
    python_requires='>=3',
)
