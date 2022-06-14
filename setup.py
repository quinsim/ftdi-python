# Standard library imports
import setuptools

# Third party imports

# Local application imports

setuptools.setup(
    name='ftdi-python',
    version="0.0.1",
    author="Quinten Simet",
    author_email="quinsim@gmail.com",
    description="",
    url="https://github.com/quinsim/ftdi-python.git",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.8.1",
        "Operating System :: Windows",
    ],
    include_package_data=True,
    install_requires=[
    ],
    extras_require={
        "dev": ["black", "pylint", "PyInstaller"],
        "test": ["pytest"],
    },
    zip_safe=False,
)