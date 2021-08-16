import re

import pytest
import RPi.GPIO as realGPIO
from unittest.mock import call

from mfrc522reader.mfrc522 import MFRC522


ANTENNA_ON = 0b00000011
ANTENNA_OFF = 0b00000000


def test_Register_read():
    # Make sure we're starting with the value we intend
    assert MFRC522.Register.CommandReg == 0x01
    # We expected the value to be shifted one place to the left changing the value to 0x02
    # Then we expect the MSB to flip to 1 which makes the value 0x82
    assert MFRC522.Register.CommandReg.read() == 0x82


def test_Register_write():
    # Make sure we're starting with the value we intend
    assert MFRC522.Register.CommandReg == 0x01
    # We expected the value to be shifted one place to the left changing the value to 0x02
    assert MFRC522.Register.CommandReg.write() == 0x02


@pytest.mark.parametrize(
    ('constructor_args', 'expected_values'),
    [
        (None, {'bus': 0, 'device': 0, 'gpio_mode': realGPIO.BOARD, 'rst_pin': MFRC522.GPIO_BOARD_RST_DEFAULT}),
        (
            {'bus': 0, 'device': 0, 'gpio_mode': realGPIO.BOARD, 'rst_pin': None},
            {'bus': 0, 'device': 0, 'gpio_mode': realGPIO.BOARD, 'rst_pin': MFRC522.GPIO_BOARD_RST_DEFAULT}
        ),
        ({'bus': 1}, {'bus': 1, 'device': 0, 'gpio_mode': realGPIO.BOARD, 'rst_pin': MFRC522.GPIO_BOARD_RST_DEFAULT}),
        ({'device': 1}, {'bus': 0, 'device': 1, 'gpio_mode': realGPIO.BOARD, 'rst_pin': MFRC522.GPIO_BOARD_RST_DEFAULT}),
        ({'gpio_mode': realGPIO.BCM}, {'bus': 0, 'device': 0, 'gpio_mode': realGPIO.BCM, 'rst_pin': MFRC522.GPIO_BCM_RST_DEFAULT}),
        ({'rst_pin': 18}, {'bus': 0, 'device': 0, 'gpio_mode': realGPIO.BOARD, 'rst_pin': 18}),
    ],
    ids=[
        'no-arg',
        'explicit defaults',
        'custom bus',
        'custom device',
        'custom gpio_mode',
        'custom rst_pin',
    ]
    )
def test_MFRC522___init__(mock_dependencies, xfer2, constructor_args, expected_values):
    initialize_card(xfer2, ANTENNA_ON)
    xfer2.set_side_effect()
    mock_dependencies.GPIO.getmode.return_value = None

    if constructor_args is None:
        reader = MFRC522()
    else:
        reader = MFRC522(**constructor_args)

    mock_dependencies.spi.open.assert_called_once_with(expected_values['bus'], expected_values['device'])
    mock_dependencies.max_speed_hz_property.assert_called_once_with(MFRC522.MAX_SPEED_HZ)
    mock_dependencies.GPIO.setmode.assert_called_once_with(expected_values['gpio_mode'])
    mock_dependencies.GPIO.setup.assert_called_once_with(expected_values['rst_pin'], mock_dependencies.GPIO.OUT)
    mock_dependencies.GPIO.output.assert_called_once_with(expected_values['rst_pin'], mock_dependencies.GPIO.HIGH)
    mock_dependencies.atexit.register.assert_called_once_with(reader.close)


@pytest.mark.parametrize(
    ('expected_status', 'expected', 'expected_rv'),
    [
        (MFRC522.ReturnCode.OK, [0x1, 0x2], True),
        (MFRC522.ReturnCode.ERR, [], False),
        (MFRC522.ReturnCode.OK, [0x1, 0x2, 0x3], False),
    ],
    ids=[
        'card present',
        'error',
        'incorrect number of bytes'
    ]
)
def test_MFRC522_card_present(reader, xfer2, expected_status, expected, expected_rv):
    clear_bits_after_collision(xfer2)
    write(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
    transceive(xfer2, [MFRC522.PICCCommand.REQA], expected, status=expected_status)
    xfer2.set_side_effect()
    rv = reader.card_present()
    assert rv == expected_rv


@pytest.mark.parametrize(
    ('card_not_present_count', 'expected', 'timeout', 'timeout_after'),
    [
        (0, '04030201', None, None),
        (7, '04030201', None, None),
        (7, '04030201', -1, None),
        (0, '04030201', 0, None),
        (1, None, 0, 1),
        (10, None, 10, 5),
        (0, None, 0, None)
    ],
    ids=[
        'success - card immediately present',
        'success - card not immediately present',
        'success - timeout -1',
        'success - timeout 0 - Card present',
        'fail - timeout 0 - Card not immediately present',
        'success - timeout 10 - Card present',
        'fail - error from anticollision',
    ]
)
def test_MFRC522_read_uid(reader_mocks, xfer2, card_not_present_count, expected, timeout, timeout_after):
    if timeout_after:
        reader_mocks.time.time.side_effect = ([0] * (timeout_after)) + [timeout]

    if not card_not_present_count:
        clear_bits_after_collision(xfer2)
        write(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
        transceive(xfer2, [MFRC522.PICCCommand.REQA], [0x0, 0x0], status=MFRC522.ReturnCode.OK)
        clear_bits_after_collision(xfer2)
        if expected is None:
            cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS1, [0x1, 0x2, 0x3, 0x4], expected_status=MFRC522.ReturnCode.TRANSCEIVE_ERROR)
        else:
            cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS1, [0x1, 0x2, 0x3, 0x4])
    else:
        if timeout_after:
            card_not_present_count -= timeout_after
            # We always do one card_present check
            card_not_present_count = max(1, card_not_present_count)
        for _ in range(card_not_present_count):
            clear_bits_after_collision(xfer2)
            write(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
            transceive(xfer2, [MFRC522.PICCCommand.REQA], [0x0, 0x0], status=MFRC522.ReturnCode.ERR)
        if not timeout_after:
            # This time we'll succeed
            clear_bits_after_collision(xfer2)
            write(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
            transceive(xfer2, [MFRC522.PICCCommand.REQA], [0x0, 0x0], status=MFRC522.ReturnCode.OK)
            clear_bits_after_collision(xfer2)
            cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS1, [0x1, 0x2, 0x3, 0x4])

    xfer2.set_side_effect()
    uid = reader_mocks.reader.read_uid(timeout)
    assert uid == expected
    do_timeout = False
    if timeout and timeout > -1:
        do_timeout = True

    if card_not_present_count != 0:
        reader_mocks.time.sleep.assert_has_calls([call(0.001)] * card_not_present_count)

    if do_timeout:
        reader_mocks.time.time.assert_has_calls([call()] * (1 + card_not_present_count))


@pytest.mark.parametrize(
    ('current_gpio_mode', 'gpio_mode', 'rst_pin', 'expected_rst_pin'),
    [
        (None, realGPIO.BOARD, None, MFRC522.GPIO_BOARD_RST_DEFAULT),
        (None, realGPIO.BCM, None, MFRC522.GPIO_BCM_RST_DEFAULT),
        (None, realGPIO.BOARD, 5, 5),
        (None, realGPIO.BCM, 5, 5),
        (realGPIO.BOARD, realGPIO.BOARD, None, MFRC522.GPIO_BOARD_RST_DEFAULT),
        (realGPIO.BOARD, realGPIO.BOARD, 5, 5),
        (realGPIO.BOARD, realGPIO.BCM, None, MFRC522.GPIO_BOARD_RST_DEFAULT),
        (realGPIO.BOARD, realGPIO.BCM, 5, None),
        (realGPIO.BCM, realGPIO.BCM, None, MFRC522.GPIO_BCM_RST_DEFAULT),
        (realGPIO.BCM, realGPIO.BCM, 5, 5),
        (realGPIO.BCM, realGPIO.BOARD, None, MFRC522.GPIO_BCM_RST_DEFAULT),
        (realGPIO.BCM, realGPIO.BOARD, 5, None),
    ],
    ids=[
        'GPIO mode unset, requested BOARD, rst_pin unset',
        'GPIO mode unset, requested BCM, rst_pin unset',
        'GPIO mode unset, requested BOARD, rst_pin set',
        'GPIO mode unset, requested BCM, rst_pin set',
        'GPIO mode BOARD, requested BOARD, rst_pin unset',
        'GPIO mode BOARD, requested BOARD, rst_pin set',
        'GPIO mode BOARD, requested BCM, rst_pin unset',
        'GPIO mode BOARD, requested BCM, rst_pin set',
        'GPIO mode BCM, requested BCM, rst_pin unset',
        'GPIO mode BCM, requested BCM, rst_pin set',
        'GPIO mode BCM, requested BOARD, rst_pin unset',
        'GPIO mode BCM, requested BOARD, rst_pin set',
    ]
    )
def test_MFRC522_setup_gpio(reader_mocks, current_gpio_mode, gpio_mode, rst_pin, expected_rst_pin):
    reader_mocks.GPIO.getmode.return_value = current_gpio_mode

    if expected_rst_pin is None:
        match = re.escape(
            f'GPIO mode ({gpio_mode}) and rst_pin ({rst_pin}) were provided but GPIO mode is already set to set to {current_gpio_mode}.'
            )
        with pytest.raises(ValueError, match=match):
            reader_mocks.reader.setup_gpio(gpio_mode, rst_pin)
    else:
        reader_mocks.reader.setup_gpio(gpio_mode, rst_pin)

        if not current_gpio_mode:
            reader_mocks.GPIO.setmode.assert_called_once_with(gpio_mode)

        reader_mocks.GPIO.setup.assert_called_once_with(expected_rst_pin, reader_mocks.GPIO.OUT)
        reader_mocks.GPIO.output.assert_called_once_with(expected_rst_pin, reader_mocks.GPIO.HIGH)


def test_MFRC522_initialize_card(reader, xfer2):
    initialize_card(xfer2, ANTENNA_ON)
    xfer2.set_side_effect()
    reader.initialize_card()


def test_MFRC522_read(reader, xfer2):
    read(xfer2, MFRC522.Register.CommandReg, 22)
    xfer2.set_side_effect()
    assert 22 == reader.read(MFRC522.Register.CommandReg)


def test_MFRC522_write(reader, xfer2):
    write(xfer2, MFRC522.Register.CommandReg, 0x77)
    xfer2.set_side_effect()
    reader.write(MFRC522.Register.CommandReg, 0x77)


@pytest.mark.parametrize(
    ('current_value', 'mask'),
    [
        (0b00000000, 0b00000000),
        (0b11111111, 0b00000000),
        (0b00001000, 0b10000000),
    ]
)
def test_MFRC522_set_bits(reader, xfer2, current_value, mask):
    set_bits(xfer2, MFRC522.Register.TxControlReg, mask, current_value)
    xfer2.set_side_effect()
    reader.set_bits(MFRC522.Register.TxControlReg, mask)


@pytest.mark.parametrize(
    ('current_value', 'mask'),
    [
        (0b00000000, 0b00000000),
        (0b11111111, 0b00000000),
        (0b00001000, 0b10000000),
    ]
)
def test_MFRC522_unset_bits(reader, xfer2, current_value, mask):
    unset_bits(xfer2, MFRC522.Register.DivIrqReg, mask, current_value)
    xfer2.set_side_effect()
    reader.unset_bits(MFRC522.Register.DivIrqReg, mask)


@pytest.mark.parametrize(
    ('spi',),
    [
        (True, ),
        (False, ),
    ],
    ids=[
        'spi set',
        'spi None',
    ]
)
def test_MFRC522_close(reader_mocks, spi):
    if not spi:
        reader_mocks.reader.spi = None
    reader_mocks.reader.close()
    if spi:
        reader_mocks.spi.close.assert_called_once()
    else:
        reader_mocks.spi.close.assert_not_called()
    reader_mocks.GPIO.cleanup.assert_called_once()


def test_MFRC522_soft_reset(reader, xfer2):
    soft_reset(xfer2)
    xfer2.set_side_effect()
    reader.soft_reset()


@pytest.mark.parametrize(
    ('antenna',),
    [
        (ANTENNA_ON, ),
        (ANTENNA_OFF, ),
    ],
    ids=['Antenna was on', 'Antenna was off']
)
def test_MFRC522_antenna_on(reader, xfer2, antenna):
    antenna_on(xfer2, antenna)
    xfer2.set_side_effect()
    reader.antenna_on()
    if antenna == ANTENNA_ON:
        assert call([MFRC522.Register.TxControlReg.write(), MFRC522.BIT_MASK_ANTENNA_POWER]) not in xfer2.spi.xfer2.mock_calls


def test_MFRC522_req_type_a(reader, xfer2):
    expected = [0x1, 0x2]
    clear_bits_after_collision(xfer2)
    write(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
    transceive(xfer2, [MFRC522.PICCCommand.REQA], expected)
    xfer2.set_side_effect()
    status, results, results_len = reader.req_type_a()
    assert status == MFRC522.ReturnCode.OK
    assert results == expected
    assert results_len == len(expected) * MFRC522.BITS_IN_BYTE


@pytest.mark.parametrize(
    ('interrupts', 'expected_status', 'expected', 'last_bits'),
    [
        (
            [0, 0, MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE],
            MFRC522.ReturnCode.OK,
            [0x01, 0xFF],
            0,
        ),
        (
            [0, 0, MFRC522.BIT_MASK_TRANCEIVE_ERRORS],
            MFRC522.ReturnCode.ERR,
            [],
            0,
        ),
        (
            [0, 0, MFRC522.BIT_MASK_LSB],
            MFRC522.ReturnCode.TIMEOUT,
            [],
            0,
        ),
        (
            [0] * MFRC522.TRANSCEIVE_CHECKS,
            MFRC522.ReturnCode.COUNTDOWN_TIMEOUT,
            [],
            0,
        ),
        (
            [0, 0, MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE],
            MFRC522.ReturnCode.COLLISION,
            [0xAB],
            0,
        ),
        (
            [0, 0, MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE],
            MFRC522.ReturnCode.OK,
            [0xAB],
            4,
        ),
        (
            [0, 0, MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE],
            MFRC522.ReturnCode.OK,
            [0xAB] * 65,
            4,
        ),
    ],
    ids=[
        'success',
        'error',
        'timeout',
        'countdown timeout',
        'collision',
        'extra bits and success',
        'too many bytes to FIFO'
        ]
)
def test_MFRC522_transceive(reader, xfer2, interrupts, expected_status, expected, last_bits):
    data = [0xF1, 0xA9, 0x27, 0x05]
    transceive(xfer2, data, expected, interrupts, expected_status, last_bits)
    xfer2.set_side_effect()
    status, results, results_len = reader.transceive(data)
    assert status == expected_status
    if len(expected) > MFRC522.FIFO_BUFFER_MAX_SIZE:
        assert results == expected[:MFRC522.FIFO_BUFFER_MAX_SIZE]
    else:
        assert results == expected
    if last_bits:
        assert results_len == ((len(expected) - 1) * MFRC522.BITS_IN_BYTE) + last_bits
    else:
        assert results_len == len(expected) * MFRC522.BITS_IN_BYTE


@pytest.mark.parametrize(
    ('uid', 'expected_results', 'expected_status', 'collision_positions'),
    [
        (
            [0x1, 0x2, 0x3, 0x4], None, MFRC522.ReturnCode.OK, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7], None, MFRC522.ReturnCode.OK, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA], None, MFRC522.ReturnCode.OK, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4], None, MFRC522.ReturnCode.OK, [3]
        ),
        (
            [0x1, 0x2, 0x3, 0x4], None, MFRC522.ReturnCode.OK, [3, 9, 12, 16, 22, 24, 31]
        ),
        (
            [0x1, 0x2, 0x3, 0x4], None, MFRC522.ReturnCode.OK, [32]
        ),
        (
            [0x1, 0x2, 0x3, 0x4], [], MFRC522.ReturnCode.CRC_ERROR, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4], None, MFRC522.ReturnCode.INVALID_COLLISION_POSITION, [33]
        ),
        (
            [0x1, 0x2, 0x3, 0x4], [1], MFRC522.ReturnCode.UNKNOWN_COLLISION_ERROR, [5, 5]
        ),
        (
            [0x1, 0x2, 0x3, 0x4], [], MFRC522.ReturnCode.TRANSCEIVE_ERROR, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4], ['too', 'many', 'items', 'in', 'result'], MFRC522.ReturnCode.INVALID_SAK_RESULT, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4], [], MFRC522.ReturnCode.SAK_CRC_ERROR, []
        ),
        (
            [0x1, 0x2, 0x3, 0x4], [-5, 34, 86, 'bogus', 'values'], MFRC522.ReturnCode.SAK_CRC_WRONG, []
        ),
    ],
    ids=[
        'success - single',
        'success - double',
        'success - triple',
        'success - single collision',
        'success - multiple collisions',
        'success - collision on 32nd bit',
        'select CRC error',
        'invalid collision position',
        'collision position does not progress',
        'transceive error',
        'invalid SAK',
        'SAK CRC calculation error',
        'SAK CRC result incorrect',
        ]
)
def test_MFRC522_anticollision(reader, xfer2, uid, expected_results, expected_status, collision_positions):
    if expected_results is None:
        # I'm lazy and I didn't want to retype it all
        expected_results = uid

    clear_bits_after_collision(xfer2)

    if len(uid) == 4:
        # This is the max cascade level we'll use
        cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS1, uid, False, collision_positions, expected_status)
    else:
        # We'll need to continue to level 2
        cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS1, [MFRC522.CASCADE_TAG] + uid[:3], True, collision_positions, expected_status)

        if len(uid) == 7:
            # This is the max cascade level we'll use
            cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS2, uid[3:], False, collision_positions, expected_status)
        else:
            # We'll need to continue to level 3
            cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS2, [MFRC522.CASCADE_TAG] + uid[3:6], True, collision_positions, expected_status)

            if len(uid) == 10:
                # need to go to CS3
                cascade_level(xfer2, MFRC522.PICCCommand.ANTICOLL_CS3, uid[6:], False, collision_positions, expected_status)

    xfer2.set_side_effect()
    status, results = reader.anticollision()
    assert status == expected_status
    assert results == expected_results


@pytest.mark.parametrize(
    ('interrupts', 'expected_status', 'expected_crc'),
    [
        (
            [0, 0, MFRC522.BIT_MASK_DIVIRQ_CRCIRQ],
            MFRC522.ReturnCode.OK,
            [0x01, 0xFF],
        ),
        (
            [0] * MFRC522.CRC_CHECKS,
            MFRC522.ReturnCode.COUNTDOWN_TIMEOUT,
            [],
        ),
    ],
    ids=['Two loops and success', 'countdown timeout']
)
def test_MFRC522_calculate_crc(reader, xfer2, interrupts, expected_status, expected_crc):
    data = [0x13, 0x15, 0xAB]
    calculate_crc(xfer2, data, expected_crc, interrupts, expected_status)
    xfer2.set_side_effect()
    status, crc = reader.calculate_crc(data)
    assert status == expected_status
    assert crc == expected_crc


@pytest.mark.parametrize(
    ('current_value',),
    [
        (0b00000000,),
        (0b11111111,),
        (0b00001000,),
    ]
)
def test_MFRC522__clear_bits_after_collision(reader, xfer2, current_value):
    clear_bits_after_collision(xfer2, current_value)
    xfer2.set_side_effect()
    reader._clear_bits_after_collision()


@pytest.mark.parametrize(
    ('data',),
    [
        ([],),
        ([0x28],),
        ([0x55, 0x19],),
    ],
    ids=['empty data', 'single element', 'two elements']
)
def test_MFRC522__write_data_to_fifo(reader, xfer2, data):
    write_fifo(xfer2, data)
    reader._write_data_to_fifo(data)


@pytest.mark.parametrize(
    ('uid', 'expected'),
    [
        ([], ''),
        ([0x04, 0x22, 0xA9, 0x45], '45A92204'),
        ([0x55, 0x19, 0xAB, 0xFF, 0x00, 0x99, 0xC1], 'C19900FFAB1955'),
        ([0xD1, 0xE9, 0x01, 0x10, 0x78, 0x26, 0x05, 0x98, 0xBB, 0x9B], '9BBB980526781001E9D1'),
        ([0xD1, 0xE9, 0x01, 0x10, 0x78, 0x26, 0x05, 0x98, 0xBB, 0x9B, 0x22, 0xFF, 0x10], '10FF229BBB980526781001E9D1'),
    ],
    ids=['empty data', 'single', 'double', 'triple', 'too many']
)
def test_MFRC522__uid_bytes_to_hex_string(reader_mocks, uid, expected):
    assert reader_mocks.reader._uid_bytes_to_hex_string(uid) == expected


def initialize_card(xfer2, antenna_value):
    soft_reset(xfer2)
    write(xfer2, MFRC522.Register.TModeReg, MFRC522.TPRESCALER_HIGH_FOUR)
    write(xfer2, MFRC522.Register.TPrescalerReg, MFRC522.TPRESCLER_LOW_EIGHT)
    write(xfer2, MFRC522.Register.TReloadRegL, MFRC522.TRELOAD_LOW_EIGHT)
    write(xfer2, MFRC522.Register.TReloadRegH, MFRC522.TRELOAD_HIGH_EIGHT)
    write(xfer2, MFRC522.Register.TxASKReg, MFRC522.ASK_MODULATION)
    write(xfer2, MFRC522.Register.ModeReg, MFRC522.MODE_DEFAULTS)
    antenna_on(xfer2, antenna_value)


def soft_reset(xfer2):
    write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.SOFT_RESET)


def antenna_on(xfer2, antenna_value):
    read(xfer2, MFRC522.Register.TxControlReg, antenna_value)
    if antenna_value == ANTENNA_OFF:
        set_bits(xfer2, MFRC522.Register.TxControlReg, MFRC522.BIT_MASK_ANTENNA_POWER, MFRC522.BIT_MASK_ANTENNA_POWER)


def write(xfer2, register, *values):
    data = [
        register.write(),
        *values
    ]
    xfer2.add(data)


def read(xfer2, register, return_value):
    data = [
        register.read(),
        0,
    ]
    xfer2.add(data, return_value)


def set_bits(xfer2, register, mask, current_value=0):
    read(xfer2, register, current_value)
    write(xfer2, register, current_value | mask)


def unset_bits(xfer2, register, mask, current_value=0):
    read(xfer2, register, current_value)
    write(xfer2, register, current_value & (~mask))


def write_fifo(xfer2, data):
    for datum in data:
        write(xfer2, MFRC522.Register.FIFODataReg, datum)


def clear_bits_after_collision(xfer2, current_value=0):
    unset_bits(xfer2, MFRC522.Register.CollReg, MFRC522.BIT_MASK_MSB, current_value)


def transceive(xfer2, data, expected, interrupts=[MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE], status=MFRC522.ReturnCode.OK, last_bits=0):
    """ Wraps up all the xfer2 calls made as part of the transceive call. Defaults to a single check loop and successful return code."""
    write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)
    unset_bits(xfer2, MFRC522.Register.ComIrqReg, MFRC522.BIT_MASK_MSB)
    set_bits(xfer2, MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)
    write_fifo(xfer2, data)
    write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)
    set_bits(xfer2, MFRC522.Register.BitFramingReg, MFRC522.BIT_MASK_MSB)
    for i in interrupts:
        read(xfer2, MFRC522.Register.ComIrqReg, i)

    if MFRC522.ReturnCode.ERR == status:
        read(xfer2, MFRC522.Register.ErrorReg, MFRC522.BIT_MASK_TRANCEIVE_ERRORS)
    elif MFRC522.ReturnCode.COLLISION == status:
        read(xfer2, MFRC522.Register.ErrorReg, MFRC522.BIT_MASK_COLLISION_ERRORS)
    elif MFRC522.ReturnCode.OK == status:
        read(xfer2, MFRC522.Register.ErrorReg, 0)
    if (MFRC522.ReturnCode.COLLISION == status) or (MFRC522.ReturnCode.OK == status):
        read(xfer2, MFRC522.Register.FIFOLevelReg, len(expected))
        read(xfer2, MFRC522.Register.ControlReg, last_bits)
        for i in range(min(MFRC522.FIFO_BUFFER_MAX_SIZE, len(expected))):
            read(xfer2, MFRC522.Register.FIFODataReg, expected[i])


def calculate_crc(xfer2, data, expected_crc, interrupts=[MFRC522.BIT_MASK_DIVIRQ_CRCIRQ], expected_status=MFRC522.ReturnCode.OK):
    """ Wraps up all the xfer2 calls made as part of the calculate_crc call. Defaults to a single check loop and successful return code."""
    write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)
    unset_bits(xfer2, MFRC522.Register.DivIrqReg, 0)
    set_bits(xfer2, MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)
    write_fifo(xfer2, data)
    write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.CALCCRC)
    for i in interrupts:
        read(xfer2, MFRC522.Register.DivIrqReg, i)
    if MFRC522.ReturnCode.OK == expected_status:
        write(xfer2, MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)
        read(xfer2, MFRC522.Register.CRCResultRegL, expected_crc[0])
        read(xfer2, MFRC522.Register.CRCResultRegH, expected_crc[1])


def cascade_level(xfer2, cascade_level, uid, set_cascade_bit=False, collision_positions=[], expected_status=MFRC522.ReturnCode.OK):
    select_crc_error = expected_status == MFRC522.ReturnCode.CRC_ERROR
    transceive_error = expected_status == MFRC522.ReturnCode.TRANSCEIVE_ERROR
    invalid_sak_result_error = expected_status == MFRC522.ReturnCode.INVALID_SAK_RESULT
    sak_crc_error = expected_status == MFRC522.ReturnCode.SAK_CRC_ERROR
    sak_crc_wrong_error = expected_status == MFRC522.ReturnCode.SAK_CRC_WRONG

    if len(uid) < 4:
        uid = [MFRC522.CASCADE_TAG] + uid

    # perform one or more collision cycles depending on how many were passed in
    known_bits = collision_cycle(xfer2, cascade_level, uid, collision_positions, set_cascade_bit)
    if known_bits is None:
        # We've run into a strange error, we're done
        return
    elif known_bits > 32:
        # This isn't a valid collision position so we're erroring out
        return
    transceive_input_uid = transceive_uid(uid, known_bits)

    # Handle if we are starting with known_bits
    if known_bits:
        uid_bytes_to_send = int(known_bits / 8)
        uid_bits_to_send = known_bits % 8
        bit_to_flip = (known_bits - 1) % 8
        index_to_flip = (uid_bytes_to_send - 1) + (1 if uid_bits_to_send else 0)
        transceive_input_uid[index_to_flip] |= (1 << bit_to_flip)
    else:
        uid_bytes_to_send = 0
        uid_bits_to_send = 0

    write(xfer2, MFRC522.Register.BitFramingReg, (uid_bits_to_send << 4) + uid_bits_to_send)

    bcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3]
    if transceive_error:
        data = [cascade_level, ((2 + uid_bytes_to_send) << 4) + uid_bits_to_send] + transceive_input_uid
        transceive(xfer2, data, None, status=MFRC522.ReturnCode.ERR)
        return
    else:
        transceive(xfer2, [cascade_level, ((2 + uid_bytes_to_send) << 4) + uid_bits_to_send] + transceive_input_uid, uid + [bcc])

    # Returned the uid so calculate the CRC for the select
    crc = [0x22, 0x56]
    data = [cascade_level, MFRC522.NVB_SEVEN_BYTES] + uid + [bcc]
    if select_crc_error:
        calculate_crc(xfer2, data, crc, interrupts=[0] * MFRC522.CRC_CHECKS, expected_status=MFRC522.ReturnCode.COUNTDOWN_TIMEOUT)
    else:
        calculate_crc(xfer2, data, crc)
        # Perform the entire select
        data += crc
        write(xfer2, MFRC522.Register.BitFramingReg, 0)
        if set_cascade_bit:
            sak = [MFRC522.BIT_MASK_CASCADE_BIT_SET] + crc
        else:
            sak = [~MFRC522.BIT_MASK_CASCADE_BIT_SET] + crc
        if invalid_sak_result_error:
            sak = ['too', 'many', 'items', 'in', 'result']
        transceive(xfer2, data, sak)
        if not invalid_sak_result_error:
            if sak_crc_error:
                calculate_crc(xfer2, sak[:1], crc, interrupts=[0] * MFRC522.CRC_CHECKS, expected_status=MFRC522.ReturnCode.COUNTDOWN_TIMEOUT)
            elif sak_crc_wrong_error:
                calculate_crc(xfer2, sak[:1], ['bogus', 'values'])
            else:
                # Recalculate the CRC on the SAK
                calculate_crc(xfer2, sak[:1], crc)


def collision_cycle(xfer2, cascade_level, uid, collision_positions, set_cascade_bit=False):
    # We can't have a collision if we only know some of the bits because if we know some of the bits, we're already in a collision cycle
    known_bits = 0
    for collision_position in collision_positions:
        # Determine what to return from the initial transceive based on the collision_position
        transceive_input_uid = transceive_uid(uid, known_bits)
        transceive_return_uid = transceive_uid(uid, collision_position)
        if known_bits:
            uid_bytes_to_send = int(known_bits / 8)
            uid_bits_to_send = known_bits % 8
            # Remember to flip the collision bit
            bit_to_flip = (known_bits - 1) % 8
            index_to_flip = (uid_bytes_to_send - 1) + (1 if uid_bits_to_send else 0)
            transceive_input_uid[index_to_flip] |= (1 << bit_to_flip)
            write(xfer2, MFRC522.Register.BitFramingReg, (uid_bits_to_send << 4) + uid_bits_to_send)
            data = [cascade_level, ((2 + uid_bytes_to_send) << 4) + uid_bits_to_send] + transceive_input_uid
            transceive(xfer2, data, transceive_return_uid, status=MFRC522.ReturnCode.COLLISION)
        else:
            # Default anticollision
            write(xfer2, MFRC522.Register.BitFramingReg, 0)
            transceive(xfer2, [cascade_level, 2 << 4], transceive_return_uid, status=MFRC522.ReturnCode.COLLISION)
        if collision_position == 32:
            read(xfer2, MFRC522.Register.CollReg, 0)
        else:
            read(xfer2, MFRC522.Register.CollReg, collision_position)
        if collision_position <= known_bits:
            # We aren't progressing (or moving backwards)
            return None
        known_bits = collision_position
    return known_bits


def transceive_uid(uid, max_bits):
    """ Slices the supplied uid List based on the maximum bits the UID should contain."""
    return uid[:int(max_bits / 8) + (1 if (max_bits % 8) else 0)]
