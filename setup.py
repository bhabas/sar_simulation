import setuptools
import versioneer

with open("README.md", "r") as fh:
    long_description = fh.read()
with open("requirements.txt", "r") as fh:
    requirements = [line.strip() for line in fh]

setuptools.setup(
    name='sar_env',
    author='Bryan Habas',
    maintainer='Bryan Habas',
    maintainer_email='BHabas@psu.edu',
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
    description='A useful module',
    long_description=long_description,
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=requirements,
    scripts=[
                'sar_general/Scripts/Control_Playground.py',
            ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Ubuntu 20.04",
    ],
)