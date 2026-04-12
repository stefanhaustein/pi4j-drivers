package com.pi4j.drivers.sensor.environment.bmx280;

import com.pi4j.io.SerialCircuitIO;
import com.pi4j.io.i2c.I2CRegisterDataReaderWriter;


/**
 * Fake I2C register access implementation that just stores the register values as sent.
 * Regular register read and write operations can be used in tests to set expected output data and
 */
public class FakeI2CRegisterDataReaderWriter implements SerialCircuitIO {

    // Allow tests direct access
    public final byte[] registerValues = new byte[256];

    @Override
    public void writeThenRead(byte[] writeData, int writeOffset, int writeLength, int delay, byte[] readData, int readOffset, int readLength) {
        if (writeLength < 1) {
            throw new IllegalArgumentException("Register expected");
        }
        int register = writeData[writeOffset] & 0xff;
        if (writeLength > 1) {
            System.arraycopy(writeData, writeOffset + 1, registerValues, register, writeLength - 1);
        }

        if (readLength > 0 && readData != null) {
            System.arraycopy(registerValues, register, readData, readOffset, readLength);
        }
    }

    @Override
    public void close() throws Exception {

    }
}
