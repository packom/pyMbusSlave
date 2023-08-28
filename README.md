# pyMbusSlave

A sample M-Bus Slave implemented in Python.

## Physical Setup

You need an M-Bus slave device like [packom.net's M-Bus Slave Hat](https://www.packom.net/m-bus-slave-hat/) connected to your Raspberry Pi.  If using the M-Bus Slave Hat, you wil need to set DEVICE to '/dev/ttyAMA0'.

## Pre-requisites

You will need the python module pySerial (not serial!).  Install it:

```
pip install pySerial
```

As both serial and pySerial import using the name serial, you may need to remove serial:

```
pip uninstall serial
```

## Downloading

```
git clone https://github.com/packom/pyMbusSlave
cd pyMbusSlave
```

## Configuring

Modify the following config at the top of slave.py:

```
# Edit these values as appropriate
DEVICE = '/dev/ttyAMA0' # Serial device controlling the slave is connected to
BAUDRATE = 2400 # Baudrate supported by this slave
ADDR = 3 # M-Bus Slave address, 0=250 are valid
ID_NO = "12345678" # M-Bus Slave serial number, can be up to 8 characters, 0-9, A-F
```

## Running

```
python slave.py
```

The slave sits and waits for commands from the M-Bus Master.  The 2 commands supported today are:
* SND_NKE (used by the Master to scan for Slaves)
* REQ_UD2 (requests the slave's user data 2)

The slave responds with hard coded information.

You can use [libmbus](https://github.com/rscada/libmbus) as the M-Bus Master software. - pyMbusSlave responds correctly to both scan and request-data commands. 

To install libmbus:

```
sudo apt install git libtool autoconf
git clone https://github.com/rscada/libmbus
cd libmbus
./build.sh
sudo make install
```

To use libmbus to get user data 2 from pyMbusSlave:

```
mbus-serial-request-data -b 2400 /dev/ttyAMA0 3 # Replace 3 with the slave's configured address

```

The data output by libmbus when querying pyMbusSlave for user data 2 is:

```
<?xml version="1.0" encoding="ISO-8859-1"?>
<MBusData>

    <SlaveInformation>
        <Id>12345678</Id>
        <Medium>Water</Medium>
        <AccessNumber>0</AccessNumber>
        <Status>00</Status>
    </SlaveInformation>

    <DataRecord id="0">
        <Function>Actual value</Function>
        <Unit>l</Unit>
        <Value>123</Value>
    </DataRecord>

    <DataRecord id="1">
        <Function>Actual value</Function>
        <Unit>reserved but historic</Unit>
        <Value>456</Value>
    </DataRecord>

</MBusData>
```

## More Information

For more information about the M-Bus protocol see the [M-Bus documentation](https://m-bus.com/documentation).

