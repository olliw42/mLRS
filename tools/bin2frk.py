import sys

# Copied from https://github.com/ExpressLRS/ExpressLRS/blob/master/src/python/opentx.py#L20 GPL-3.0 and modified
def gen_frsky(source, target):
    with open(source, "rb") as _in:
        with open(target, "wb+") as _out:
            source_content = _in.read()
            # append FrSky header (16bytes)
            '''
                struct FrSkyFirmwareInformation_no_pack {
                    uint32_t fourcc; = 0x4B535246
                    uint8_t headerVersion; = 1
                    uint8_t firmwareVersionMajor;
                    uint8_t firmwareVersionMinor;
                    uint8_t firmwareVersionRevision;
                    uint32_t size; == len(bin_content)
                    uint8_t productFamily;
                    uint8_t productId;
                    uint16_t crc;
                };
            '''
            print("Bin size: %u" % len(source_content))
            _out.write(b"\x46\x52\x53\x4B") # fourcc
            _out.write(b"\x01") # header version
            _out.write(b"\x00\x00\x00")  # fw versions
            size = len(source_content) + 16
            _out.write(bytearray(
                [size & 0xFF, size >> 8 & 0xFF, size >> 16 & 0xFF, size >> 24 & 0xFF]))
            _out.write(b"\x00\x00") # productFamily, productId
            _out.write(b"\x00\x00") # crc
            _out.write(source_content)
            _out.close()

def main():
    if len(sys.argv) != 3:
        print("usage ./bin2frk.py <source bin> <target bin>")
        return
    gen_frsky(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()

