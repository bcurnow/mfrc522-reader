<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [mfrc522-reader](#mfrc522-reader)   
- [Why not just use the existing mfrc522 library?](#why-not-just-use-the-existing-mfrc522-library)   
- [Known Issues or Untested Functionality](#known-issues-or-untested-functionality)   
- [Missing Functionality](#missing-functionality)   
- [NXP Semiconductors documentation](#nxp-semiconductors-documentation)   
- [Usage](#usage)   
   - [Constructor Arguments](#constructor-arguments)   
   - [Methods](#methods)   
      - [read_uid](#read_uid)   
      - [card_present](#card_present)   
      - [Additional Methods](#additional-methods)   
- [Running tests](#running-tests)   

<!-- /MDTOC -->
# mfrc522-reader

A Python implementation of an MFRC522 SPI reader

This is a Python reimplementation mainly based on (https://github.com/miguelbalboa/rfid) but also influenced by (https://github.com/pimylifeup/MFRC522-python). Their work in turn was based on others so lots of solid work that I based my library on.

# Why not just use the existing mfrc522 library?
In short, because it doesn't work correctly.

Here's the longer answer. The [mfrc522 Python library](https://github.com/pimylifeup/MFRC522-python) works great for reading PICCs with a single sized uid (4 bytes). Unfortunately, it does not implement the cascade tag/cascade level support correctly for double or triple tags. I believe this is due to the fact that's based on https://github.com/miguelbalboa/rfid, which doesn't implement that logic correctly either. Specifically, their code does not properly handle the first step of the anticollision process for cascade levels 2 and 3 where only the cascade level and NVB should be sent in the request. Instead, it tries to send additional uid bytes which the MFRC522 doesn't recognize (because they're either zero or part of the first 4 bytes, either way, not valid).

# Known Issues or Untested Functionality
I've written tests to ensure that the library performs as intended in various scenarios (e.g. single, double, triple uid sizes, collisions, etc.). However, while I wrote the code and tested it, I only had access to single and double type A PICCs. I believe the code works properly with triple size uids (and therefore cascade level 3) but I haven't been able to actually verify with a PICC.

The other area that is difficult to integration test is a collision. I haven't been able to simulate a collision with multiple PICCs so while the code matches the spec, there's a possibility it's not 100% correct.

# Missing Functionality

This library implements only the commands for the MFRC522 that I cared about for my project, specifically, reading the uid off a Disney MagicBand. I specifically do not implement any of the MiFare data reading and encryption portions (as Disney sets the access bits so I don't have access to that part of the band anyway). The low level functions are mostly there so feel free to fork and implement those yourself.

This library only implements support for type A PICCs. Again, the building blocks are all there for the type B commands, I just haven't implemented them because I don't have a need to read type B PICCs.

Unlike https://github.com/miguelbalboa/rfid, I don't have support for starting the anticollision/select phase with an arbitrary cascade level and number of valid bits. Because of that, there's some code missing that deals with the cascade tag (CT). Since I don't support starting mid-process, I'm never in a position where I need to intentionally add the cascade tag to the data. While the code correctly handles the cases where the PICC provides the cascade tag (and cascade bit in the SAK), my code always starts with no information and at cascade level 1. If you have a well known UID (or a well known part of a UID) and want to be able to select that specific PICC, you'll have to modify the `MFRC522.anticollision` method.

# NXP Semiconductors documentation

I leveraged the excellent [NXP Semiconductors MRFC522 data sheet](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf) extensively to both expand my understanding of how the MFRC522 works as well as provide documentation and examples within the code. If you're not familiar with the MFRC522 itself, this is a great place to start.

# Usage

The intended usage is very simple, start by importing the MFRC522 class:
```
from mfrc522reader import MFRC522
```

Then, to read the uid, simply construct an instance and call `read_uid()`:
```
reader = MFRC522()
reader.read_uid()
```

## Constructor Arguments

The constructor for the MFRC522 class takes the following arguments, all of which are optional:
* `bus` - Passed directly to the [spidev](https://pypi.org/project/spidev/) library which uses it to construct the device name for the MFRC522 (e.g. `/dev/spidev<bus>.<device>`). Default: `0`
* `device` - Passed directly to the [spidev](https://pypi.org/project/spidev/) library which uses it to construct the device name for the MFRC522 (e.g. `/dev/spidev<bus>.<device>`). Default: `0`
* `gpio_mode` - Passed directly to the [RPi.GPIO](https://pypi.org/project/RPi.GPIO/) library. This controls the meaning of pin numbers. Default: `RPi.GPIO.BOARD`
* `rst_pin` - Identifies the reset pin. The default value will auto set the pin based on the `gpio_mode` (`15` for `RPi.GPIO.BCM` and `22` for `RPi.GPIO.BOARD`). Default: `None`

Please note that the constructor does not take a speed argument. The speed is always set the the maximum supported by the MFRC522 which is `106 kBd`.

## Methods

### read_uid

The `read_uid` method implements a loop which checks for a type A PICC (card) in the RF field and, if found, starts the anticollision/select process and returns the UID. This method implements a timeout and will sleep `.001` seconds between each check for a card.

The method takes the following arguments:
* `timeout` - The number of seconds to wait for a PICC to enter the field. If None or any value less than zero (0), will never timeout. If set to zero (0), it will check for a card exactly once and, if not found, will immediately timeout. Default: `None`

The methods returns the uid read by the anticollision/select process as a reversed hex string. Each byte in the uid is converted to a two hex digit string using uppercase letters in the reverse byte order the card returns it. This is because I also have a second reader that I use for testing that isn't an MFRC522 which returns the UID in that order and I needed the values to be the same.

### card_present

The `card_present` methods implements the check for a type A PICC in the RF field. It does this by executing a single `REQA` command and converting the result to a boolean.

The method takes no arguments.

The method returns `True` if there is a type A PICC in the RF field or `False` in all other cases.

### Additional Methods

The class implements additional, lower-level methods (e.g. `soft_reset`, `antenna_on`, `req_type_a`, `transceive`) but I'll leave you to the documentation in the docstrings if you want to understand those.

# Running tests

I prefer to create a Docker container to run all my testing in. A (Dockerfile) is provided along with some helper scripts. I don't publish these containers to a registry as they're designed to be run locally as part of the testing.

There are two helper scripts:
* `scripts/build.sh` - Builds a new Docker container. This script will name the container based on the parent directory (typically mfrc522-reader) and always tags as `latest`. Will create a new user named `mfrc522` mapped to the current uid on the host which will allow the parent dir of the Dockerfile to be bind mounted into the container but not mangle the permissions. Will also copy files from the `docker-files` directory into the mfrc522 users home to configure a few OS settings (path, vim) to allow for some basic troubleshooting inside the container.
* `scripts/docker.sh` - Runs the container and drops into a bash shell

Once inside the container, run the following commands to execute testing and coverage:
```
pip install -e .
coverage run -m pytest
```
