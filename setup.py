#!/usr/bin/env python

from setuptools import find_packages, setup


with open('README.md', 'r') as fh:
    long_description = fh.read()

setup(
    name='mfrc522-reader',
    version='1.0.2',
    author='Brian Curnow',
    author_email='brian.curnow+mfrc522-reader@gmail.com',
    description='An implementation of a reader for NXP Semiconductors MFRC522 RFID readers.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/bcurnow/mfrc522-reader',
    packages=find_packages(),
    classifiers=[
        'Programming Language :: Python :: 3.9',
        'License :: OSI Approved :: Apache Software License'
        'Operating System :: OS Independent',
        'Intended Audience :: Developers',
        'Natural Language :: English',
        'Topic :: Software Development :: Libraries :: Python Modules',

    ],
    python_requires='>=3.6',
    install_requires=[
        'RPi.GPIO',
        'spidev',
    ],
)
