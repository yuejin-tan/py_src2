import Ui_mainWin
from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import functools
import datetime
import time

from canlib import canlib, Frame
import struct

VERSION_STR = "kw21 CAN_GUI Ver 0.1"

# 比特率
cdbBitrate = canlib.Bitrate.BITRATE_500K


# sg cmds
SG_cmd = 0
SG_pow = 0
SG_pSlop = 100

# rx frame decode str
rxFrameDecStr1 = ""
rxCnt1 = 0
rxFrameDecStr2 = ""
rxCnt2 = 0


class mainWindow(QtWidgets.QMainWindow, Ui_mainWin.Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.actioncdb_GUI.triggered.connect(functools.partial(
            QtWidgets.QMessageBox.aboutQt, None, "about cdb_GUI"))

        self.statusBar().showMessage(VERSION_STR, 2000)

        # 界面部分

        self.pushButton.clicked.connect(self.btnSlot)

        # 实例化定时器
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timerSlot)
        self.timer.start(100)

        # 是否发送
        self.isComModeOn = 0

    def btnSlot(self):
        global ch
        if (self.isComModeOn):
            # 已经是打开状态，开始关闭
            self.isComModeOn = 0
            self.pushButton.setText("stop->start")
            self.timer.stop()
            ch.busOff()

        else:
            # 关闭状态，打开
            self.isComModeOn = 1
            self.pushButton.setText("start->stop")
            self.timer.start(self.spinBox.value())
            ch.busOn()

    def updateTxCmd(self):
        global SG_cmd
        global SG_pow
        global SG_pSlop

        if (self.radioButton.isChecked()):
            SG_cmd = 0
        elif (self.radioButton_2.isChecked()):
            SG_cmd = 0x11
        elif (self.radioButton_3.isChecked()):
            SG_cmd = 0x22
        elif (self.radioButton_4.isChecked()):
            SG_cmd = 0x33
        elif (self.radioButton_5.isChecked()):
            SG_cmd = 0x44
        elif (self.radioButton_6.isChecked()):
            SG_cmd = 0x55
        SG_pow = self.doubleSpinBox.value()
        SG_pSlop = self.doubleSpinBox_2.value()

    def updateRxDecStr(self):
        global rxFrameDecStr1
        global rxFrameDecStr2
        self.textEdit.setText(rxFrameDecStr1)
        self.textEdit_2.setText(rxFrameDecStr2)

    def timerSlot(self):
        if (self.isComModeOn):
            # 接收
            revStatusUtil()
            self.updateRxDecStr()
            # 发送
            self.updateTxCmd()
            sendCmdUtil()


def enumPorts():
    # 枚举CAN端口
    print(datetime.datetime.now().strftime(
        '[%H:%M:%S.%f]')+" enum CAN Ports: ", end="")
    num_channels = canlib.getNumberOfChannels()
    print(f"Found {num_channels} channels")
    for ch in range(num_channels):
        chd = canlib.ChannelData(ch)
        print(f"{ch}. {chd.channel_name} ({chd.card_upc_no} / {chd.card_serial_no})")


# 读取变量的辅助函数


def revStatusUtil():
    global ch
    global rxFrameDecStr1
    global rxFrameDecStr2
    global rxCnt1
    global rxCnt2

    revCnt = 0

    while (1):

        try:
            frame = ch.read(timeout=20)
            revCnt += 1
            if (frame.flags != canlib.MessageFlag.EXT or frame.dlc != 8):
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" frame type err!")
            else:
                if (frame.id == 0x0C101070):
                    # 帧1
                    rxCnt1 += 1
                    rxFrameDecStr1 = datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+"\tcnt1:\t"+str(rxCnt1)+"\r\n"
                    rxFrameDecStr1 += "1.1-spd:\t{:.2f}\r\n".format(
                        struct.unpack("<H", frame.data[0:2])[0])
                    rxFrameDecStr1 += "1.2-Udc:\t{:.2f}\r\n".format(
                        struct.unpack("<H", frame.data[2:4])[0]*0.1)
                    rxFrameDecStr1 += "1.3-Idc:\t{:.2f}\r\n".format(
                        struct.unpack("<H", frame.data[4:6])[0]*0.1-100)
                    rxFrameDecStr1 += "1.4-Tmotor:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[6:7])[0]-30)
                    rxFrameDecStr1 += "1.5-Tpower:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[7:8])[0]-30)

                elif (frame.id == 0x14201070):
                    # 帧2
                    rxCnt2 += 1
                    rxFrameDecStr2 = datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+"\tcnt2:\t"+str(rxCnt2)+"\r\n"
                    rxFrameDecStr2 += "2.1-Is:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[0:1])[0]-100)
                    rxFrameDecStr2 += "2.2-Iq:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[1:2])[0]-100)
                    rxFrameDecStr2 += "2.3-Id:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[2:3])[0]-100)
                    rxFrameDecStr2 += "2.4-Te:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[3:4])[0]*0.1-11)
                    rxFrameDecStr2 += "2.5-Pdc:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[4:5])[0]-30)
                    rxFrameDecStr2 += "2.5-Tctrl:\t{:.2f}\r\n".format(
                        struct.unpack("<B", frame.data[5:6])[0]-30)
                    rxFrameDecStr2 += "2.6-errFlg:\t0x" + \
                        str(frame.data[6:7].hex())+" 0x" + \
                        str(frame.data[7:8].hex())

                else:
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" frame Id err!")

        except Exception as e:
            if (revCnt):
                break
            else:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" rev fail!")
                print(e)
                break


# 设置变量的辅助函数


def sendCmdUtil():
    global ch

    try:
        txFrameData = struct.pack(
            "<B", SG_cmd) + struct.pack("<H", int(SG_pow)) + struct.pack(
                "<H", int(SG_pSlop)) + struct.pack("<B", 0) + struct.pack("<H", 0)

        txFrame = Frame(id_=0x04107010, data=txFrameData, dlc=8,
                        flags=canlib.MessageFlag.EXT)
        ch.write(txFrame)
        ch.writeSync(timeout=20)

    except Exception as e:
        print(datetime.datetime.now().strftime(
            '[%H:%M:%S.%f]')+" send fail!")
        print(e)


if __name__ == "__main__":
    print(
        "\n"+datetime.datetime.now().strftime('[%H:%M:%S.%f] ')+VERSION_STR+" sdardle!\n")
    # 枚举端口
    enumPorts()

    # 尝试打开端口
    # 保证变量全局性？
    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=cdbBitrate,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    # ch.busOn()

    app = QtWidgets.QApplication(sys.argv)
    fontx = QtGui.QFont()
    fontx.setPixelSize(14)
    app.setFont(fontx)

    w = mainWindow()
    w.show()

    ret = app.exec()

    if (w.isComModeOn):
        ch.busOff()
    ch.close()

    sys.exit(ret)
