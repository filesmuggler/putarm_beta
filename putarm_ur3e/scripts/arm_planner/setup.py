import setuptools

#with open("README.md", "r") as fh:
#    long_description = fh.read()

setuptools.setup(
    name='arm_planner',
    version='0.1',
    scripts=['arm_planner'],
    author="Piotr Kicki",
    author_email="piotr.kicki@put.poznan.pl",
    description="An arm_planner package",
#    long_description=long_description, 
    long_description_content_type="text/markdown",
    url="---",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "License :: OSI Approved :: MIT License", 
        "Operating System :: OS Independent",
    ],
 )
