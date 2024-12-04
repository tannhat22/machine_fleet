from hostlink import HostLink
import struct
import time

# Địa chỉ IP của 3 máy tiện M151:
# 192.168.1.97 -> 99


def hostLink_test(plctype, ip, port):
    pyplc = HostLink(plctype)
    pyplc.connect(ip, port)
    # check access to word unit
    # pyplc.write_data("DM20001", "U", 1000)
    while True:
        dk_doc = pyplc.read_data("DM4000", ".U")
        if dk_doc == 1:
            values_u = pyplc.continuous_read_data("DM4002", 6, ".U")
            values_f = pyplc.continuous_read_data("DM4008", 12, ".L")
            isOK = pyplc.read_data("DM4032", ".U")
            print(
                f"Times: {values_u[2]}/{values_u[1]}/{values_u[0]} - {values_u[3]}:{values_u[4]}:{values_u[5]}\n"
                f"  MES_UAMPL_INI: {struct.unpack("f", struct.pack("I", values_f[0]))[0]}\n"
                f"  MES_UPHAL_INI: {struct.unpack("f", struct.pack("I", values_f[1]))[0]}\n"
                f"  MES_UAMPE_INI: {struct.unpack("f", struct.pack("I", values_f[2]))[0]}\n"
                f"  MES_UPHAE_INI: {struct.unpack("f", struct.pack("I", values_f[3]))[0]}\n"
                f"  MES_UAMPS_INI: {struct.unpack("f", struct.pack("I", values_f[4]))[0]}\n"
                f"  MES_UPHAS_INI: {struct.unpack("f", struct.pack("I", values_f[5]))[0]}\n"
                f"  MES_UAMPL: {struct.unpack("f", struct.pack("I", values_f[6]))[0]}\n"
                f"  MES_UPHAL: {struct.unpack("f", struct.pack("I", values_f[7]))[0]}\n"
                f"  MES_UAMPE: {struct.unpack("f", struct.pack("I", values_f[8]))[0]}\n"
                f"  MES_UPHAE: {struct.unpack("f", struct.pack("I", values_f[9]))[0]}\n"
                f"  MES_UAMPS: {struct.unpack("f", struct.pack("I", values_f[10]))[0]}\n"
                f"  MES_UPHAS: {struct.unpack("f", struct.pack("I", values_f[11]))[0]}"
            )
            if isOK:
                print("Danh gia: OK!")
            else:
                print("Danh gia: NG!")

            print("//////////////////////////////////////////////\n")
            pyplc.write_data("DM4001", ".U", 1)
            pyplc.write_data("DM4000", ".U", 0)
        time.sleep(0.1)

    # check access to bit unit
    # odd size test
    # pyplc.write_data("MR1000", "U", 1)
    # value = pyplc.read_data("MR1000", "U")
    # print(value)

    # check access to word units
    # pyplc.continuous_write_data("DM20010", 5, "U", [1, 2, 3, 4, 5])
    # value = pyplc.continuous_read_data("DM4008", 1, ".L")
    # print(value)

    # check access to bit units
    # odd size test
    # pyplc.continuous_write_data("MR10001", 5, "U", [1, 1, 0, 0, 1])
    # value = pyplc.continuous_read_data("MR10001", 5, "U")
    # print(value)

    # pyplc.change_mode_CPU(1)


if __name__ == "__main__":
    plctype, ip, port = ["KV", "192.168.1.97", 8501]
    hostLink_test(plctype, ip, port)
