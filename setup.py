"""A setuptools based setup module.

See:
https://packaging.python.org/en/latest/distributing.html
https://github.com/pypa/sampleproject
"""

from setuptools import setup, find_packages

# To use a consistent encoding
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, "README.rst"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="pigpio-ads1x15",
    use_scm_version=True,
    setup_requires=["setuptools_scm"],
    description="pigpio library for controlling an ADS1x15 ADC. Forked from https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    # The project's main homepage.
    url="https://github.com/CommReteris/pigpio_ads1x15",
    # Author details
    author="Lorenzo Seirup",
    author_email="commreteris@gmail.com",
    install_requires=["pigpio"],
    # Choose your license
    license="MIT",
    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Hardware",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
    ],
    # What does your project relate to?
    keywords="adafruit ads1x115 adc hardware pigpio",
    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    packages=["ppads1x15"],
)
