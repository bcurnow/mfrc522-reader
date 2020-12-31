from types import SimpleNamespace

import pytest
import RPi.GPIO as realGPIO
from unittest.mock import call, patch, PropertyMock

from mfrc522reader.mfrc522 import MFRC522


class Xfer2:
    """ Maintains parallel lists of calls to spi.xfer2 and the expected return values """
    def __init__(self, spi):
        self.spi = spi
        self.calls = []
        self.side_effect = []

    def add(self, input, output=0):
        self.calls.append(call(input))
        self.side_effect.append([None, output])

    def assert_calls(self):
        self.spi.xfer2.assert_has_calls(self.calls, any_order=False)
        assert self.spi.xfer2.call_count == len(self.calls)

    def set_side_effect(self):
        """ This method must be called last and will setup side_effect for the xfer2 calls."""
        self.spi.xfer2.side_effect = self.side_effect


class Xfer2Debug:
    """ Prints more useful call and response information for debugging tests."""
    def __init__(self, xfer2):
        self.xfer2 = xfer2

    def print_expected_vs_actual(self):
        headers = ('Expected', 'Actual')
        print(f'{headers[0]:<60}{headers[1]:<60}')
        meaningful_expected = self._convert_to_meaning(self.xfer2.calls, self.xfer2.side_effect)
        meaningful_actual = self._convert_to_meaning(self.xfer2.spi.xfer2.mock_calls, self.xfer2.side_effect)
        print(f'Call Count: {len(meaningful_expected):<48}Call Count: {len(meaningful_actual):<48}')
        for expected, actual in zip(meaningful_expected, meaningful_actual):
            if expected == actual:
                print(f'{expected:<60}{actual:<60}')
            else:
                print(f'*{expected:<60}{actual:<60}')

    def _convert_to_meaning(self, calls, side_effect):
        """ Converts the List of calls to values with actual meaning rather than simple decimal values."""
        meaning = []
        for acall, rv in zip(calls, side_effect):
            if type(acall.args) is tuple:
                # There is only a single argument to the xfer2 call, a list
                if len(acall.args) == 1:
                    # Grab the list
                    args = acall.args[0]
                    if type(args) is list:
                        # OK, now we can actually start checking
                        # First argument in the register to the print to, need to determine if this is a read or a write
                        register = args[0]
                        xfer2_type = 'write'
                        data = args[1:]
                        if (register & MFRC522.BIT_MASK_MSB) != 0:
                            xfer2_type = 'read'
                            # This is a read (MSB is 1) so drop the MSB
                            register &= ~MFRC522.BIT_MASK_MSB
                            # We don't care about the data as it's always a single zero so drop it
                            data = []

                        # Convert back into a MFRC522.Register by shifting one to the left to undo the shift in read/write
                        register = MFRC522.Register(register >> 1)

                        # Convert the rest of the data elements
                        for i, datum in enumerate(data):
                            if isinstance(datum, MFRC522.PCDCommand) or isinstance(datum, MFRC522.PICCCommand):
                                # Replace any commands with their names
                                data[i] = datum.name
                            elif isinstance(datum, int):
                                # Negative numbers we'll leave in decimal
                                if datum >= 0:
                                    data[i] = f'0x{datum:X} ({datum}) (0b{datum:08b})'
                            else:
                                data[i] = '?' + data[i]

                        if data:
                            if len(data) > 1:
                                meaning.append(f'{xfer2_type}({register.name}, {data})')
                            else:
                                meaning.append(f'{xfer2_type}({register.name}, {data[0]})')
                        else:
                            meaning.append(f'{xfer2_type}({register.name}) (0x{rv[1]:X} ({rv[1]}) (0b{rv[1]:08b})')

                    else:
                        meaning.append(f'Wrong type of call.args[0] ({type(args)}): {acall}')
                else:
                    meaning.append(f'Wrong number of args: {acall}')
            else:
                meaning.append(f'Unknown type of args ({type(acall.args)}): {acall}')
            if acall.kwargs:
                meaning.append(f'Found kwargs in call, no support yet (should not happen!): {acall}')
        return meaning


@pytest.fixture
def mock_dependencies():
    """ This fixture provides mocked out versions of all the external dependencies for the mfrc522 module ."""
    with patch('mfrc522reader.mfrc522.spidev') as spidev, \
         patch('mfrc522reader.mfrc522.GPIO') as GPIO, \
         patch('mfrc522reader.mfrc522.atexit') as atexit, \
         patch('mfrc522reader.mfrc522.time') as time:

        spi = spidev.SpiDev.return_value
        # Setup property mocks for the properties we use
        max_speed_hz_property = PropertyMock('max_speed_hz', return_value=MFRC522.MAX_SPEED_HZ)
        type(spi).max_speed_hz = max_speed_hz_property
        type(GPIO).BCM = PropertyMock('BCM', return_value=realGPIO.BCM)
        type(GPIO).BOARD = PropertyMock('BOARD', return_value=realGPIO.BOARD)
        type(GPIO).OUT = PropertyMock('OUTPUT', return_value=realGPIO.OUT)
        type(GPIO).HIGH = PropertyMock('HIGH', return_value=realGPIO.HIGH)

        yield SimpleNamespace(** {
            'spidev': spidev,
            'spi': spi,
            'max_speed_hz_property': max_speed_hz_property,
            'GPIO': GPIO,
            'atexit': atexit,
            'time': time,
         })


@pytest.fixture
def reader_mocks(mock_dependencies):
    """
    This fixture runs with all the external dependencies mocked out and additionally provides a
    constructed MFRC522 with all the mocks reset after constructed.
    """
    reader = MFRC522()
    # Reset all the mocks so that we start clean
    for name in vars(mock_dependencies):
        mock = getattr(mock_dependencies, name)
        mock.reset_mock()
    mock_dependencies.reader = reader
    yield mock_dependencies


@pytest.fixture
def reader(reader_mocks):
    """ Same as reader_mocks but only yields the reader object, not the entire namespace. """
    yield reader_mocks.reader


@pytest.fixture
def xfer2(mock_dependencies):
    """ Sets up an Xfer2 instance and ensures it is asserted after the test. """
    xfer2 = Xfer2(mock_dependencies.spi)
    yield xfer2
    try:
        xfer2.assert_calls()
    except AssertionError as e:
        print('The assertion failed, printing debug:')
        Xfer2Debug(xfer2).print_expected_vs_actual()
        raise e from None
