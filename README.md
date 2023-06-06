# pyMbusSlave

A sample M-Bus Slave implemented in Python.

## Physical Setup

You need an M-Bus slave device like [packom.net's M-Bus Slave Hat](https://www.packom.net/m-bus-slave-hat/) connected to your Raspberry Pi.  If using the M-Bus Slave Hat, you wil need to set DEVICE to '/dev/ttyAMA0'.

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

Alternatively, you can override these values on the command line.

### Virtual serial port

On Linux and Unix systems, specify `pty` as the serial port to create a
virtual serial port.  The script will report the name of the actual virtual
serial port you can connect to.

```
RC=0 stuartl@rikishi /tmp/pyMbusSlave $ python3 slave.py --device pty
INFO:pyMbusSlave:pyMbusSlave
INFO:pyMbusSlave:  Serial device:   pty
INFO:pyMbusSlave:  Baudrate:        2400
INFO:pyMbusSlave:  Slave address:   3
INFO:pyMbusSlave:  Slave device ID: 12345678
INFO:pyMbusSlave:Point your M-Bus master at /dev/pts/4
```

Then point your M-Bus master at this device:

```
RC=0 stuartl@rikishi /tmp/libmbus/bin $ ./mbus-serial-scan /dev/pts/4
Found a M-Bus device at address 3
RC=0 stuartl@rikishi /tmp/libmbus/bin $ ./mbus-serial-request-data /dev/pts/4 3
<?xml version="1.0" encoding="ISO-8859-1"?>
<MBusData>

    <SlaveInformation>
        <Id>12345678</Id>
        <Manufacturer>TST</Manufacturer>
        <Version>1</Version>
        <ProductName></ProductName>
        <Medium>Other</Medium>
        <AccessNumber>0</AccessNumber>
        <Status>00</Status>
        <Signature>0000</Signature>
    </SlaveInformation>

    <DataRecord id="0">
        <Function>Instantaneous value</Function>
        <StorageNumber>0</StorageNumber>
        <Unit> V</Unit>
        <Value>1234</Value>
        <Timestamp>2023-06-05T01:14:26Z</Timestamp>
    </DataRecord>

    <DataRecord id="1">
        <Function>Instantaneous value</Function>
        <StorageNumber>0</StorageNumber>
        <Unit>Volume ( m^3)</Unit>
        <Value>456</Value>
        <Timestamp>2023-06-05T01:14:26Z</Timestamp>
    </DataRecord>

</MBusData>
```

### TCP Socket

Specify `tcp:${PORT}` or `tcp:${ADDRESS}:${PORT}` (IPv6 should work here too)
and it'll create a simulated M-Bus/TCP socket.

```
stuartl@LPA075:~/vrt/projects/widesky/edge/mbus/privdoc/pyMbusSlave$ python3 slave.py --device tcp:20000
INFO:pyMbusSlave:pyMbusSlave
INFO:pyMbusSlave:  Serial device:   tcp:20000
INFO:pyMbusSlave:  Baudrate:        2400
INFO:pyMbusSlave:  Slave address:   3
INFO:pyMbusSlave:  Slave device ID: 12345678
DEBUG:pyMbusSlave:Will bind to any address port 20000
INFO:pyMbusSlave:Listening ...
```

```
stuartl@LPA075:~/vrt/projects/widesky/edge/mbus/privdoc/libmbus$ bin/mbus-tcp-request-data localhost 20000 3
<?xml version="1.0" encoding="ISO-8859-1"?>
<MBusData>

    <SlaveInformation>
        <Id>12345678</Id>
        <Manufacturer>TST</Manufacturer>
        <Version>1</Version>
        <ProductName></ProductName>
        <Medium>Other</Medium>
        <AccessNumber>0</AccessNumber>
        <Status>00</Status>
        <Signature>0000</Signature>
    </SlaveInformation>

    <DataRecord id="0">
        <Function>Instantaneous value</Function>
        <StorageNumber>0</StorageNumber>
        <Unit> V</Unit>
        <Value>1234</Value>
        <Timestamp>2023-06-06T21:27:49Z</Timestamp>
    </DataRecord>

    <DataRecord id="1">
        <Function>Instantaneous value</Function>
        <StorageNumber>0</StorageNumber>
        <Unit>Volume ( m^3)</Unit>
        <Value>456</Value>
        <Timestamp>2023-06-06T21:27:49Z</Timestamp>
    </DataRecord>

</MBusData>
```

## Running

```
python slave.py
```

The configuration can be overridden using command line arguments:

* `--device`: Serial port device for the slave.
* `--baud`: Baud rate of the serial interface.
* `--addr`: Primary address of the slave
* `--id`: Serial number of the slave
* `--manuf`: Manufacturer code

The slave sits and waits for commands from the M-Bus Master.  The 2 commands supported today are:
* `SND_NKE` (used by the Master to scan for Slaves)
* `REQ_UD2` (requests the slave's user data 2)

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

