"""
This code is heavily influenced by https://github.com/pimylifeup/MFRC522-python and https://github.com/miguelbalboa/rfid but has been adjusted
so that it's better commented and suited specifically for what this library requires (e.g. reading the uid off a tag).
The additional documentation comes from NXP's data-sheet on the MFRC522: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf.
"""
import atexit
from enum import IntEnum, unique
import time

import RPi.GPIO as GPIO
import spidev


class MFRC522:
    """
    For reference, here are a couple of terms/acronyms:
      - ATQA  ->   Answer To reQuest A, the name of the bytes that come back from the REQA command.
      - CRC   ->   Cyclic Redundancy Check, a typical error-detecting code which indicates if any of the data has changed.
      - FIFO  ->   First In, First Out, the first data value provided will be the first value processed
      - LSB   ->   Least Significant Bit, aka low-order bit or right-most bit is the which determines whether a binary number is odd or even.
                   The LSB is typically in the 2^1 postion, for examle in 0000 00001 it's a one (1), in 1111 1110 it's a zero (0).
                   While it is possible for the LSB to be either the farthest left or right, this code is setup for the LSB to be the farthest right.
      - MSB   ->   Most Significant Bit, aka high-order or left-most bit is the highest value bit position in a binary number.
                   The MSB is typically in the 2^8 position, for example, in 1000 0000 it's a one (1), in 0111 1111 it's a zero (0).
                   While it is possible for the MSB to be either the farthest left or right, this code is setup for the MSB to be the farthest left.
      - NVB   ->   Number of valid bits, the total number of bytes (high 4 bits) and bits (low 4 bits) we will are sending, this is for the complete
                   command and not just the UID
      - PCB   ->   Proximity Coupling Device, the RFID reader hardware
      - PICC  ->   Proximity Integrated Circuit Card
      - SAK   ->   Select acknowledgement, what is returned by the PICC for a full select
      - SEL   ->   Select, the actual value of this command changes based on the current level (1, 2, 3)
      - SPI   ->   Serial Peripheral Interface, a synchronous serial communication interface and how this code expects the MFRC522 to be connected.
    """

    @unique
    class Register(IntEnum):
        # starts and stops command execution
        CommandReg = 0x01
        # interrupt request bits
        ComIrqReg = 0x04
        # interrupt request bits for CRC
        DivIrqReg = 0x05
        # error bits showing the error status of the last command executed
        ErrorReg = 0x06
        # Status register for various commands
        Status1Reg = 0x07
        # input and output of 64 byte FIFO buffer
        FIFODataReg = 0x09
        # number of bytes stored in the FIFO buffer
        FIFOLevelReg = 0x0A
        # miscellaneous control registers
        ControlReg = 0x0C
        # adjustments for bit-oriented frames
        BitFramingReg = 0x0D
        # bit-collision detection
        CollReg = 0x0E
        # defines general modes for transmitting and receiving
        ModeReg = 0x11
        # controls the logical behavior of the antenna driver pins TX1 and TX2
        TxControlReg = 0x14
        # controls the setting of the transmission modulation
        TxASKReg = 0x15
        # The MSB of the CRC result
        CRCResultRegH = 0x21
        # The LSB of the CRC result
        CRCResultRegL = 0x22
        # timer settings
        TModeReg = 0x2A
        # defines settings for the internal timer
        TPrescalerReg = 0x2B
        # defines the 16-bit timer reload MSB value
        TReloadRegH = 0x2C
        # defines the 16-bit timer reload LSB value
        TReloadRegL = 0x2D

        def read(self):
            """
            Returns the register as an appropriately bit-shifted value with the first bit indicating a read operation.

            Registers are represented by a 6-bit value where the LSB is always 0 (zero)
            and the MSB indicates the mode: 0 for write, 1 for read.
            We use the write method first and then bitwise OR with 1000 0000
            to ensure that the MSB is 1.
            """
            return self.write() | MFRC522.BIT_MASK_MSB

        def write(self):
            """
            Returns the register as an appropriately bit-shifted value with the first bit indicating a write operation.

            Registers are represented by a 6-bit value where the LSB is always 0 (zero)
            and the MSB indicates the mode: 0 for write, 1 for read.
            To ensure that the LSB is always zero, we bit shift 1 place to the left.
            To ensure that the MSB is 0, we bitwise AND with 0111 1110
            Example:
              register = 0x03 = 0000 0011
              bit shift << 1 = 0000 0110
              AND with 0111 1110

                0000 0110
              & 0111 1110
              -----------
                0000 0110

            If you now drop the MSB and LSB you're left with 000011 which is still 0x03
            """
            return (self << 1) & MFRC522.BIT_MASK_REGISTER

    @unique
    class PCDCommand(IntEnum):
        # Stop any in process commands and idle the board
        IDLE = 0x00
        # Perform a CRC check
        CALCCRC = 0x03
        # Tranceive data
        TRANSCEIVE = 0x0C
        # Perform a soft reset of the board
        SOFT_RESET = 0x0F

    @unique
    class ReturnCode(IntEnum):
        """ The actual values don't matter, the names are important."""
        # everything is OK
        OK = 0
        # There was no tag to read
        NOTAGERR = 1
        # Something went wrong (e.g. parity error, buffer oveflow, protocol error)
        ERR = 2
        # The timer completed before we were done
        TIMEOUT = 3
        # Our countdown completed without the timer going off
        COUNTDOWN_TIMEOUT = 4
        # A collision error happened
        COLLISION = 5
        # A collision happened but the position is within the data we already know which shouldn't be possible
        UNKNOWN_COLLISION_ERROR = 6
        # A collision happened by the result did not provide a valid collision position
        INVALID_COLLISION_POSITION = 7
        # We successfully completed a SEL but the SAK wasn't the right size/content
        INVALID_SAK_RESULT = 8
        # We got a SAK but the CRC value didn't actually check out
        SAK_CRC_WRONG = 9
        # We got an error from a transceive call, this is never returned by the transceive method itself
        TRANSCEIVE_ERROR = 10
        # We got an error from a calculate_crc call, this is never returned by the calculate_crc method itself
        CRC_ERROR = 11
        # We got an error from the calculate_crc call used to re-calculate the SAK, this is never returned by the calculate_crc method itself
        SAK_CRC_ERROR = 12

    @unique
    class PICCCommand(IntEnum):
        # Is there a type A card in the field?
        REQA = (0x26)
        # Anti-collision cascade level 1
        ANTICOLL_CS1 = 0x93
        # Anti-collision cascade level 2
        ANTICOLL_CS2 = 0x95
        # Anti-collision cascade level 3
        ANTICOLL_CS3 = 0x97

    # Force a 100% ASK modulation by setting [6] = 1
    ASK_MODULATION = 0b01000000
    # Set TxLastBits to 7 which is the short frame format
    BIT_FRAMING_SHORT_FRAME_FORMAT = 0b00000111
    # A bit mask for [0] and [1] (Tx1RFEn, Tx2RFEn) which indicate the current power state of the antenna
    BIT_MASK_ANTENNA_POWER = 0b00000011
    # Bit mask to check the SAK for the cascade bit
    BIT_MASK_CASCADE_BIT_SET = 0b000000100
    # A bit mask for collision [3] errors
    BIT_MASK_COLLISION_ERRORS = 0b00001000
    # A bit mask that pulls CollPosNotValid [5]
    BIT_MASK_COLLREG_POSITION_NOT_VALID = 0b00100000
    # A bit amsk that pulls CollPos [0]-[4]
    BIT_MASK_COLLREG_POSITION = 0b00011111
    # Bit mask for the RX (receive) and Idle (command complete) interrupts
    BIT_MASK_COMIRQ_RX_AND_IDLE = 0b00110000
    # Bit mask for the CRCIRq flag in the DivIrqReg
    BIT_MASK_DIVIRQ_CRCIRQ = 0b00000100
    # The least signficant bit
    BIT_MASK_LSB = 0b00000001
    # The most significant bit
    BIT_MASK_MSB = 0b10000000
    # A bit mask for the six-digit register values (dropping MSB and LSB)
    BIT_MASK_REGISTER = 0b01111110
    # A bit mask for protocol [0], partity [1], or buffer overflow [4] errors
    BIT_MASK_TRANCEIVE_ERRORS = 0b00010011
    # Turns on/masks all the interrupts except for HiAlertIEn ([3])
    BIT_MASK_TRANSCEIVE_IRQ = 0b11110111
    # The number of bits in a byte
    BITS_IN_BYTE = 8
    # The byte value that represents a cascade tag (CT)
    # If this is the first byte of a UID, it indicates that the PICC has more bytes for the UID
    # and we need to move to the next cascade level to get them
    CASCADE_TAG = 0x88
    # Indicates a collision in the 32nd bit
    COLLISION_POSITION_32 = 0b00000000
    # The size (in bytes) of the FIFO buffer
    FIFO_BUFFER_MAX_SIZE = 64
    # The default RST pin (GPIO 15) when gpio_mode is set to GPIO.BCM
    GPIO_BCM_RST_DEFAULT = 15
    # The default RST pin (GPIO 25) when gpio_mode is set to GPIO.BOARD (the default)
    GPIO_BOARD_RST_DEFAULT = 22
    # 106 kBd - see initialize_card()
    MAX_SPEED_HZ = 106000
    # Default value for the ModeReg
    # Set the CRC preset value to 0x6363 [0]-[1]
    # Set the polarity of MFIN to HIGH [3]
    # Ensure the transmitter can only be started if an RF field is generated [5]
    # Set MSBFirst to false
    # Other bits are reserved and therefore left at 0
    MODE_DEFAULTS = 0b00111101
    # We will be transferring all 7 possible bytes (command, NVB, plus all 5 uid bits)
    NVB_SEVEN_BYTES = 0b01110000
    # The following values configure the timer so the result is a timer delay of 0.025 seconds (~25 miliseconds)
    # This done by setting the TPrescaler (12-bits) first 4 bits to 0 and the second 8 to 169 which results in a total value of 169
    # Then the TReload (16-bits) first 8 bits are set to 3 and the second 8 bits are set 232 which results in a total value of 1000
    # To get to the delay seconds, the following equation is used:
    # (TPrescaler * 2 + 1) * (TReload + 1)
    # ------------------------------------
    #     13.56 Mhz (13,560,000)
    # For our values this ends up as:
    # (169 * 2 + 1) * (1000 + 1) OR
    # 339 * 1001 OR
    # 339,339
    # ----------
    # 13560000
    # Which is 0.025025 of a second or 25 miliseconds
    # NOTE: According to the NXP docs, 339 clock cycles results in a delay of 25 microseconds
    #       A value of 169 for TPrescaler results in 339 in the equation so the way to think about this is:
    #       The TPrescaler represents 25 micro seconds the TReload represents how many 25 microsecond time-slots
    #       to count before triggering the timer IRQ
    # Sets the value of TPrescaler's high 4 bits (out of 12) (value = 0)
    TPRESCALER_HIGH_FOUR = 0b10000000
    # Sets the value of the TPrescaler's low 8 bits (out of 12) (value = 169)
    TPRESCLER_LOW_EIGHT = 0b10101001
    # The high 8 bits of the TReload value (value = 3)
    TRELOAD_HIGH_EIGHT = 0b00000011
    # The low 8 bits of the TReload value (value = 232)
    TRELOAD_LOW_EIGHT = 0b11101000
    # Map between the cascade level and the number of bytes in the UID
    # This is mostly because I don't like a method with if/else to do this
    UID_SIZE_AT_CASCADE_LEVEL = {
        PICCCommand.ANTICOLL_CS1: 4,
        PICCCommand.ANTICOLL_CS2: 7,
        PICCCommand.ANTICOLL_CS3: 10,
    }
    # This is the number of IRQ checks to make for the TRANSCEIVE command.
    # This value is based on the TReload value as this indicates how many 25 microsecond delays.
    # We want to check the IRQ twice any many times to ensure that our IRQ checking loop is
    # longer than the timer interval as each time through the loop should be ~ 25 microsecond
    # To use the TReload value, we need to combine the high and low bits so we shift the
    # high eight bits 8 places to the left and then add in the low bits. This results in
    # the total value of TReload (e.g. 1000) and allows us to base this constant on a multiple
    # of that.
    TRANSCEIVE_CHECKS = 2 * ((TRELOAD_HIGH_EIGHT << 8) + TRELOAD_LOW_EIGHT)
    # See TRANSCEIVE_CHECKS for an explanation (we want more time for CRC)
    CRC_CHECKS = 5 * ((TRELOAD_HIGH_EIGHT << 8) + TRELOAD_LOW_EIGHT)

    def __init__(self, bus=0, device=0, gpio_mode=GPIO.BOARD, rst_pin=None):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = MFRC522.MAX_SPEED_HZ
        self.setup_gpio(gpio_mode, rst_pin)
        # Make sure we cleanup before we exit
        atexit.register(self.close)
        self.initialize_card()

    def card_present(self):
        """ Returns true if there are any type A PICCs in the field, False otherwise. """
        status, results, results_len = self.req_type_a()

        # According to the NXP docs the content of the ATQA should be ignored
        # However, we will check both the status and that we received 16 bits (2 bytes)
        if status == MFRC522.ReturnCode.OK and results_len == 16:
            return True
        return False

    def read_uid(self, timeout=None):
        """
        Reads the UID from a PICC.

        Args:
            timeout: The number of seconds to wait for a read.
                     Accepts floating point numbers (e.g. 1.5, .0001).
                     If the value is None or a negative number, will wait infinitely.
                     If the value is zero, will look for a card and read the UID only once before timing out
                     Defaults to None

        Returns:
            The uid as a hex string if a type A PICC was found and None in all other cases.
        """
        do_timeout = False
        # timeout = 0 is valid!
        if timeout is not None and timeout > -1:
            do_timeout = True
            timeout = time.time() + timeout

        while True:
            if self.card_present():
                status, results = self.anticollision()

                if status == MFRC522.ReturnCode.OK:
                    return self._uid_bytes_to_hex_string(results)
                else:
                    # An error happened, return None
                    return None
            time.sleep(0.001)  # Wait a very short time to avoid hogging the CPU
            if do_timeout and time.time() >= timeout:
                return None

    def setup_gpio(self, gpio_mode, rst_pin):
        """ Correctly configures the GPIO library."""
        current_gpio_mode = GPIO.getmode()

        if current_gpio_mode:
            # Something already set the GPIO mode
            if current_gpio_mode != gpio_mode:
                # The value is different that we expected
                # Check to see if the rst_pin was passed in, if it wasnt', we'll just adopt the
                # current GPIO mode
                if rst_pin:
                    # Can't continue, the reset pin was passed in and the GPIO modes don't align
                    # if we continue, we'll setup the reset pin in the wrong place.
                    msg = [
                        f'GPIO mode ({gpio_mode}) and rst_pin ({rst_pin}) were provided but GPIO mode is already set to set to {current_gpio_mode}. ',
                        'Either change the rst_pin to match the current GPIO mode or determine what other process is setting the GPIO mode.'
                    ]
                    raise ValueError(''.join(msg))
                else:
                    # rst_pin is the default (None) so we can just switch over to the other mode
                    gpio_mode = current_gpio_mode
        else:
            # GPIO mode is currently unset, set it to the value we wasnt
            GPIO.setmode(gpio_mode)

        if not rst_pin:  # auto set based on GPIO mode
            if gpio_mode == GPIO.BCM:
                rst_pin = MFRC522.GPIO_BCM_RST_DEFAULT
            else:
                rst_pin = MFRC522.GPIO_BOARD_RST_DEFAULT

        # Setup the reset pin as an output
        GPIO.setup(rst_pin, GPIO.OUT)
        GPIO.output(rst_pin, GPIO.HIGH)

    def initialize_card(self):
        """ Resets the card and then sets our defaults."""
        self.soft_reset()
        self.write(MFRC522.Register.TModeReg, MFRC522.TPRESCALER_HIGH_FOUR)
        self.write(MFRC522.Register.TPrescalerReg, MFRC522.TPRESCLER_LOW_EIGHT)
        self.write(MFRC522.Register.TReloadRegL, MFRC522.TRELOAD_LOW_EIGHT)
        self.write(MFRC522.Register.TReloadRegH, MFRC522.TRELOAD_HIGH_EIGHT)
        self.write(MFRC522.Register.TxASKReg, MFRC522.ASK_MODULATION)
        self.write(MFRC522.Register.ModeReg, MFRC522.MODE_DEFAULTS)
        self.antenna_on()

    def read(self, register):
        """
        Reads a value from the card and return the byte read.

        Args:
            register: The MFRC522.Register to read from.

        """
        val = self.spi.xfer2([register.read(), 0])
        # The above reads a single byte but based on the NXP docs, the first byte returns is always discarded
        return val[1]

    def write(self, register, values):
        """
        Writes a list of values to the card.

        Args:
            register: The MFRC522.Register to write to.
            values: A List of bytes to write.
        """
        self.spi.xfer2([register.write(), values])

    def set_bits(self, register, mask):
        """
        Reads the current value and then sets the specified bits to 1.

        Args:
            register: The MFRC522.Register to set the bits on.
            mask: A byte representing the bitmask to use.
        """
        cur_val = self.read(register)
        # bitwise or (|) will ensure that all bit posision set to 1 in the mask are set to 1 in the current value
        self.write(register, cur_val | mask)

    def unset_bits(self, register, mask):
        """
        Reads the current value and then sets the specified bits to 0.

        Args:
            register: The MFRC522.Register to set the bits on.
            mask: A byte representing the bitmask to use.
                  The current value will be AND'd with the complement (NOT)
        """
        cur_val = self.read(register)
        # bitwise and (&) with a NOT'd (~) maks will ensure that all bit positions set to 1 in the mask are set to 0 in the current value
        self.write(register, cur_val & (~mask))

    def close(self):
        if self.spi:
            self.spi.close()
        # Make sure to reset the value of any GPIO pins we've used to be a good citizen
        GPIO.cleanup()

    def soft_reset(self):
        """ Executes a soft reset on the card to reset all registers to default values (internal buffer is not changed)."""
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.SOFT_RESET)

    def antenna_on(self):
        cur_val = self.read(MFRC522.Register.TxControlReg)
        # Determine if the antenna is currently off, if it is, turn it back on
        if (cur_val & MFRC522.BIT_MASK_ANTENNA_POWER) != MFRC522.BIT_MASK_ANTENNA_POWER:
            self.set_bits(MFRC522.Register.TxControlReg, MFRC522.BIT_MASK_ANTENNA_POWER)

    def req_type_a(self):
        """
        Checks to see if type A PICC's are in the field.

        Returns:
            A tuple containing an MFRC522.ReturnCode,
            a List of bytes representing the result,
            and an int representing the length of the result.
            If the return value is not MFRC522.ReturnCode.OK,
            the result is typically an empty list and the length is zero
        """
        self._clear_bits_after_collision()
        self.write(MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
        return self.transceive([MFRC522.PICCCommand.REQA])

    def transceive(self, data):
        """
        Sends a command and its associated data to the PCD for execution.

        Args:
            data: A List of bytes. The first byte is expected to be a
                  MFRC522.PCDCommand or a MFRC522.PICCCommand

        Returns:
            A tuple containing an MFRC522.ReturnCode,
            a List of bytes representing the result,
            and an integer representing the length of the result.
            If the return value is not MFRC522.ReturnCode.OK,
            the result is typically an empty list and the length is zero
        """
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Set the Set1 bit ([7]) to 0 which clears the IRQ register
        self.unset_bits(MFRC522.Register.ComIrqReg, MFRC522.BIT_MASK_MSB)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)

        self._write_data_to_fifo(data)

        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)

        # Set StartSend [7] to 1 to start the transmission of data
        self.set_bits(MFRC522.Register.BitFramingReg, MFRC522.BIT_MASK_MSB)

        # Setup default return values
        results = []
        results_len = 0

        countdown = MFRC522.TRANSCEIVE_CHECKS
        while countdown > 0:
            countdown -= 1
            interrupts = self.read(MFRC522.Register.ComIrqReg)
            if (interrupts & MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE):
                # Either receiving is complete or the command completed, either way, we're done waiting
                break

            if (interrupts & MFRC522.BIT_MASK_LSB):
                # timer interrupt - nothing received
                return (MFRC522.ReturnCode.TIMEOUT, results, results_len)

        if countdown == 0:
            # None of the interrupts fired before the countdown finished
            # Did we lose connectivity with the MFRC522?
            return (MFRC522.ReturnCode.COUNTDOWN_TIMEOUT, results, results_len)

        errors = self.read(MFRC522.Register.ErrorReg)
        if errors & MFRC522.BIT_MASK_TRANCEIVE_ERRORS:
            return (MFRC522.ReturnCode.ERR, results, results_len)

        # Check how many bytes were written to the FIFO
        bytes_written = self.read(MFRC522.Register.FIFOLevelReg)
        # Check [0][1][2] to see how many valid bits in the last byte (0 (zero) indicates the whole byte is valid)
        bits_in_last_byte = self.read(MFRC522.Register.ControlReg) & 0b00000111
        if bits_in_last_byte != 0:
            results_len = (bytes_written - 1) * MFRC522.BITS_IN_BYTE + bits_in_last_byte
        else:
            results_len = bytes_written * MFRC522.BITS_IN_BYTE

        if bytes_written > MFRC522.FIFO_BUFFER_MAX_SIZE:
            bytes_written = MFRC522.FIFO_BUFFER_MAX_SIZE

        for i in range(bytes_written):
            results.append(self.read(MFRC522.Register.FIFODataReg))

        if errors & MFRC522.BIT_MASK_COLLISION_ERRORS:
            return (MFRC522.ReturnCode.COLLISION, results, results_len)

        # Normally, you'd want to check if a CRC was requested
        # However, I don't need to actually read MiFare data (which is when this is requested)
        # So I didn't implement that check

        return (MFRC522.ReturnCode.OK, results, results_len)

    def anticollision(self):
        """
        Performs the anticollision/select process to identify the uid of a type A PICC.

        This method is called anticollision but it actually implements the entire anticollision/select process.
        There are three cascade levels to work through (1, 2, 3) depending on the size of the uid (4-, 7-, or 10-bytes).
        The process is started with no information about the PICCs and at cascade level 1.
        An initial request is made and, if there are no collisions between PICCs (or there's only one in the field),
        we can move directly onto a select operation.
        If this succeeds, we check the SAK to see if the PICC indicated there were more bytes in the uid.
        If there are, we move to the next cascade level and start the process again.
        If not, we have the whole uid and it is returned.

        Returns:
            A tuple containing an MFRC522.ReturnCode and a List bytes.
            If the first value is MFRC522.ReturnCode.OK, the List contains 4, 7 or 10 bytes representing the UID
            If the first value is any other ReturnCode the List is generally the results of the last PCD command or empty
        """
        self._clear_bits_after_collision()

        # How many bits within the UID have we verified so far
        known_bits = 0
        cascade_level = MFRC522.PICCCommand.ANTICOLL_CS1
        uid = [0] * 10
        # A list to hold the data we need to transceive
        buffer = [0] * 9
        # How many extra bits do we need to transceive?
        transceive_bits = 0
        # How many slots in the buffer do we need to transceive (this will control the size of the slice later)
        # Default to 2 as the first command is always just SEL + NVB
        transceive_buffer_size = 2

        # Perform the steps below until the SAK indicates we have the complete UID
        while True:
            # Determine some additional options based on the current cascade level
            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS1:
                uid_start_index = 0  # We know nothing yet

            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS2:
                uid_start_index = 4  # We know about 4 bytes

            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS3:
                uid_start_index = 7  # We know about 7 bytes

            # Set the command we'll be using
            buffer[0] = cascade_level

            # Determine which index in the buffer we need to start copying the uid information into
            # Default to 2 ([0] = SEL, [1] = NVB)
            buffer_index = 2

            # At the start of each cascade level we know nothing about the uid and therefore none of the bits are valid
            known_bits = 0
            valid_bits = 0
            # Start the SEL/ANTICOLL loop
            select_finished = False
            select = False
            while not select_finished:
                if select:  # We've got all the bits we're going to get for this cascade level, time to select
                    buffer[1] = MFRC522.NVB_SEVEN_BYTES  # SEL, NVB, 4 bytes of UID (or CT + 3 bytes) and the BCC
                    # Calculate the Block Check Character (BCC) (buffer[6])
                    buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]

                    # Calculate a CRC_A value on the first 7 bytes of the buffer
                    status, crc_results = self.calculate_crc(buffer[:7])
                    if status != MFRC522.ReturnCode.OK:
                        return (MFRC522.ReturnCode.CRC_ERROR, crc_results)

                    # Add CRC to the buffer
                    buffer[7] = crc_results[0]
                    buffer[8] = crc_results[1]

                    # We have all the bytes so no extraneous bits to transceive
                    transceive_bits = 0
                    # We're sending a full select so we'll use all 9 bytes
                    transceive_buffer_size = 9
                else:  # This is anticollision
                    if valid_bits > 0:
                        transceive_bytes = int(valid_bits / 8)
                        transceive_bits = valid_bits % 8
                        # Calculate the total number of whole bytes we're going to send,
                        # we always send SEL and NVB and we're only sending the valid UID bytes
                        nvb_byte_count = 2 + transceive_bytes
                        # Set the NVB - high 4 is equal to the total bytes, low 4 is equal to the remaining bits
                        buffer[1] = (nvb_byte_count << 4) + transceive_bits
                        transceive_buffer_size = nvb_byte_count + (1 if transceive_bits else 0)
                    else:
                        # This is the first anticollision request for this level
                        # Only sending SEL and NVB
                        transceive_bits = 0
                        buffer[1] = (2 << 4) + transceive_bits
                        transceive_buffer_size = 2

                # Reset the the bit-oriented frame settings
                # This is made up of rxAlign ([4], [5], [6]) and indicates where the LSB is and
                # TxLastBits ([0], [1], [2]) which stores how many bits in the last byte will be transmitted
                # [3] is reserved and will be left at zero
                # [7] is the start send indicator and will be left at zero since we're not yet ready to start
                self.write(MFRC522.Register.BitFramingReg, (transceive_bits << 4) + transceive_bits)

                # Transceive only the part of the buffer indicated by transceive_buffer_size!
                status, results, results_len = self.transceive(buffer[:transceive_buffer_size])

                if status == MFRC522.ReturnCode.COLLISION:
                    # There was more than PICC in the field! Read the CollReg to get more info
                    collision_info = self.read(MFRC522.Register.CollReg)
                    if collision_info & MFRC522.BIT_MASK_COLLREG_POSITION_NOT_VALID:
                        # We don't have a valid collision position and can't continue
                        return (MFRC522.ReturnCode.INVALID_COLLISION_POSITION, results)
                    collision_position = collision_info & MFRC522.BIT_MASK_COLLREG_POSITION
                    if (collision_position == 0):
                        collision_position = 32
                    if collision_position <= valid_bits:
                        # Wait a second, we already know these bits are valid so
                        # something has gone terribly wrong
                        return (MFRC522.ReturnCode.UNKNOWN_COLLISION_ERROR, results)

                    # Copy the new information we just got from the results into the buffer for use in the next iteration.
                    # buffer_index is set to the index containing the first bit of the UID
                    for i in range(len(results)):
                        buffer[buffer_index + i] = results[i]
                    # Based on the results of collision, we now have some additional known bits
                    known_bits = collision_position
                    # The protocol indicates we need to flip the bit prior to the collision position and try again
                    # Need to first know the specific bit that caused the collision
                    collision_bit = known_bits % 8
                    # Need to know the specific bit location within the last byte to flip
                    # Also need to account for any byte boundaries so we can't just subtract 1 from the collision bit
                    bit_to_flip = (known_bits - 1) % 8
                    # Determine which index in the buffer contains the bit that needs to be flipped
                    # Start with index 1 ([0] = SEL, [1] = NVB), add the number of whole bytes and then add one if there are still some bits
                    bit_to_flip_index = 1 + (int(known_bits / 8)) + (1 if collision_bit else 0)
                    # Flip the bit by bitwise OR'ing with 1 shifted to the correct bit position
                    buffer[bit_to_flip_index] |= (1 << bit_to_flip)
                    # The known bits have all be validated so reset valid_bits
                    valid_bits = known_bits
                elif status != MFRC522.ReturnCode.OK:
                    return (MFRC522.ReturnCode.TRANSCEIVE_ERROR, results)
                else:
                    # We were successful, but sucessful at what?
                    if select:
                        # we just performed a select and it's successful so we're done
                        select_finished = True
                    else:
                        # we just performed an anticollision without error so results contains all 32 bits of the UID for this level
                        # Copy them over to the buffer
                        # We can blindly copy starting at index 2 ([0] = SEL, [1] = NVB) as the cascade tag isn't a factor
                        # the response is 40 bits, the first 32 are the uid bytes, the last 8 are the BCC value which we can discard
                        for i in range(len(results) - 1):
                            buffer[2 + i] = results[i]
                        known_bits = 32
                        # make sure we perform a select this time through
                        select = True

            # We've completed the select for this cascade level, copy over the known uid bytes
            # Need to adjust based on whether we've received a cascade tag or not
            if buffer[2] == MFRC522.CASCADE_TAG:
                buffer_index_with_uid = 3
                bytes_to_copy = 3
            else:
                buffer_index_with_uid = 2
                bytes_to_copy = 4
            for i, buffer_index_with_uid in zip(range(bytes_to_copy), range(buffer_index_with_uid, buffer_index_with_uid + bytes_to_copy)):
                uid[uid_start_index + i] = buffer[buffer_index_with_uid]

            # Select complete, let's review the SAK
            if (len(results) != 3):
                # We don't have 1 byte of SAK and 2 bytes of CRC
                return (MFRC522.ReturnCode.INVALID_SAK_RESULT, results)

            # Let's double check that CRC_A we got back by recalculating our own
            status, crc_results = self.calculate_crc(results[:1])
            if status != MFRC522.ReturnCode.OK:
                return (MFRC522.ReturnCode.SAK_CRC_ERROR, crc_results)
            if (results[1] != crc_results[0]) or (results[2] != crc_results[1]):
                return (MFRC522.ReturnCode.SAK_CRC_WRONG, results + crc_results)

            # Do we have the whole UID or not?
            if results[0] & MFRC522.BIT_MASK_CASCADE_BIT_SET:
                # Nope, there's still more
                # Take advantage of the fact that MFRC522.PICCCommand is an IntEnum and each cascade level is +2 from the previous
                cascade_level = MFRC522.PICCCommand(cascade_level + 2)
            else:
                return (MFRC522.ReturnCode.OK, uid[:MFRC522.UID_SIZE_AT_CASCADE_LEVEL[cascade_level]])

    def calculate_crc(self, data):
        """
        Instructs the PCB to calculate a CRC value on the provided data.

        Args:
            data: A List of bytes to calculate the CRC value on.

        Returns:
            A MFRC522.ReturnCode and a List of 2 bytes representing the CRC value.
            If the return code is not MFRC522.ReturnCode.OK the List is the results
            of the last command or an empty List.
        """
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Clear the CRC IRQ bit
        self.unset_bits(MFRC522.Register.DivIrqReg, MFRC522.BIT_MASK_DIVIRQ_CRCIRQ)
        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)
        self._write_data_to_fifo(data)

        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.CALCCRC)

        countdown = MFRC522.CRC_CHECKS
        while countdown > 0:
            countdown -= 1
            interrupts = self.read(MFRC522.Register.DivIrqReg)
            if interrupts & MFRC522.BIT_MASK_DIVIRQ_CRCIRQ:
                # Stop calculation, we're done
                self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)
                rv = []
                rv.append(self.read(MFRC522.Register.CRCResultRegL))
                rv.append(self.read(MFRC522.Register.CRCResultRegH))
                return (MFRC522.ReturnCode.OK, rv)
        # Timeout happened
        return (MFRC522.ReturnCode.COUNTDOWN_TIMEOUT, [])

    def _clear_bits_after_collision(self):
        """ Sets ValuesAfterColl in the CollReg to zero to ensure all bits are cleared after a collision."""
        self.unset_bits(MFRC522.Register.CollReg, MFRC522.BIT_MASK_MSB)

    def _write_data_to_fifo(self, data):
        """ Writes the List of bytes to the FIFO register. """
        for datum in data:
            self.write(MFRC522.Register.FIFODataReg, datum)

    def _uid_bytes_to_hex_string(self, uid):
        """
        Converts the UID to a hex string.

        Args:
            uid: A List of 4, 7 or 10 bytes representing the individual UID values

        Returns:
            A string containing each of the bytes in two digit HEX format and in reverse order.
            The order is reversed as I'm attempting to duplicate the string provided by the evdev-based
            reader which returns the values in that same order.
        """
        rv = []
        for digit in reversed(uid):
            # f-string format indicates to pad the value with up to 2 zeros (0>2)
            # and use uppercase hex values
            rv.append(f'{digit:0>2X}')
        return ''.join(rv)
