import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()
with open("requirements.txt", "r") as fh:
    requirements = [line.strip() for line in fh]

setuptools.setup(
   name='crazyflie_env',
   version='3.0',
   description='A useful module',
   long_description=long_description,
   author='Bryan Habas',
   packages=setuptools.find_packages(),
   python_requires=">=3.6",
   install_requires=requirements,
   scripts=[
            'sar_general_scripts/Control_Playground.py',
           ]
)