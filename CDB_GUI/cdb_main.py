import Ui_mainWIn
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

VERSION_STR = "CDB_GUI Ver 0.1"

paraSetPreDefName = [
    "",  # 1
    "",  # 2
    "",  # 3
    "",  # 4
    "",  # 5
    "",  # 6
    "",  # 7
    "",  # 8
    "",  # 9
]

trigSrcPreDefName = [
    "CH1_Udc",  # 1
    "",  # 2
]

logSrcPreDefName = [
    "CH1_Udc",  # 1
    "CH1_ifbk.V",  # 2
    "omegaMfbk",  # 3
    "thetaRDC_raw",  # 4
    "",  # 5
    "",  # 6
    "",  # 7
    "",  # 8
    "",  # 9
    "",  # 10
    "",  # 11
    "",  # 12
    "",  # 13
    "",  # 14
    "",  # 15
    "",  # 16
]

# 主中断频率
IsrFreq = 40e3


def setupVarDict():
    global varDict
    global varTypeDict

    # 手动补充
    varDict[""] = 0

    tiMapFileLoc = """C:/Users/t/workspace_v12/P21kw_v2_SiC_S/build_release/P21kw_v2_SiC_S.map"""

    with open(tiMapFileLoc, 'r') as f:
        mapFileTxt = f.read()

    startMapChaStr = """GLOBAL SYMBOLS: SORTED BY Symbol Address"""

    mapFileTxt = mapFileTxt[mapFileTxt.find(
        startMapChaStr)+len(startMapChaStr):-1]

    mapFileTxtLines = mapFileTxt.split('\n')

    # 扔掉前面4行和后面3行
    mapFileTxtLines = mapFileTxtLines[4:-2]

    # 创建变量表字典
    for metaTxt in mapFileTxtLines:
        metaTxtList = metaTxt.split()
        if (not metaTxtList[2].startswith("_")):
            continue
        if (metaTxtList[2][1:].startswith("_")):
            continue
        metaTxtList[2] = metaTxtList[2][1:]
        varAddr = int(metaTxtList[1], 16)
        if (varAddr < 0x18000):
            continue
        if (varAddr < 0x3F800 and varAddr >= 0x19000):
            continue
        if (varAddr >= 0x40000):
            continue
        varDict[metaTxtList[2]] = varAddr

    # 人工标注非float型的变量
    varTypeDict["CH1_cur_mode"] = "int16_t"
    varTypeDict["CH2_cur_mode"] = "int16_t"
    varTypeDict["speed_mode"] = "int16_t"
    varTypeDict["channel_mode"] = "int16_t"
    varTypeDict["CH1_angle_mode"] = "int16_t"
    varTypeDict["CH2_angle_mode"] = "int16_t"
    varTypeDict["thetaRDC_raw_offset_CH1"] = "uint16_t"
    varTypeDict["thetaRDC_raw_offset_CH2"] = "uint16_t"
    varTypeDict["thetaRDC_raw"] = "uint16_t"
    varTypeDict["IProtectFlg_CH1"] = "int16_t"
    varTypeDict["IProtectFlg_CH2"] = "int16_t"
    varTypeDict["SPDProtectFlg"] = "int16_t"
    varTypeDict["UdcProtectFlg_CH1"] = "int16_t"
    varTypeDict["UdcProtectFlg_CH2"] = "int16_t"
    varTypeDict["tempProtectFlg"] = "int16_t"
    varTypeDict["isr_start_pwm_cnt"] = "uint16_t"
    varTypeDict["isr_end_pwm_cnt"] = "uint16_t"
    varTypeDict["targetWaveMode"] = "int16_t"
    varTypeDict["CH1_Iv_raw"] = "int16_t"
    varTypeDict["CH1_Iw_raw"] = "int16_t"
    varTypeDict["CH1_Idc_raw"] = "int16_t"
    varTypeDict["CH1_Iv_raw_offset"] = "int16_t"
    varTypeDict["CH1_Iw_raw_offset"] = "int16_t"
    varTypeDict["CH1_Idc_raw_offset"] = "int16_t"
    varTypeDict["CH2_Iu_raw"] = "int16_t"
    varTypeDict["CH2_Iv_raw"] = "int16_t"
    varTypeDict["CH2_Idc_raw"] = "int16_t"
    varTypeDict["CH2_Iu_raw_offset"] = "int16_t"
    varTypeDict["CH2_Iv_raw_offset"] = "int16_t"
    varTypeDict["CH2_Idc_raw_offset"] = "int16_t"
    varTypeDict["ad2s1210ErrGPIO"] = "uint16_t"
    varTypeDict["dbg_ad2s1210Err"] = "uint16_t"
    varTypeDict["ad2s1210WaitCnt"] = "int16_t"
    varTypeDict["ADRC_mode"] = "uint16_t"
    varTypeDict["cpu2mainIsrTick"] = "uint16_t"
    varTypeDict["RDV_err_check"] = "function"
    varTypeDict["adcOffset_init"] = "function"
    varTypeDict["scd_Udc_set"] = "function"
    varTypeDict["sram_init"] = "function"
    varTypeDict["ad2s1210_ClaAccModeEn"] = "function"
    varTypeDict["ad2s1210_ClaAccModeEn"] = "function"
    varTypeDict["ad2s1210_ClaAccModeEn"] = "function"
    varTypeDict["ad2s1210_ClaAccModeEn"] = "function"

    # 硬编码触发用的变量, 先摆烂用 uint16_t ,只读取, 不影响
    varDict["cdb1.trigSta"] = varDict["cdb1"]
    varTypeDict["cdb1.trigSta"] = "uint16_t"

    # 解析结构体, 暂时只对纯float结构体
    stru2Dtab_PIctrl_struct = [
        ["kp", 0],
        ["ki", 2],
        ["kb", 4],
        ["max", 6],
        ["min", 8],
        ["integral", 10],
        ["ans", 12],
    ]

    stru2Dtab_Trans_struct = [
        ["U", 0],
        ["V", 2],
        ["W", 4],
        ["al", 6],
        ["be", 8],
        ["d", 10],
        ["q", 12],
        ["abdq0", 14],
    ]

    addStructVarUtil("UdcPI", stru2Dtab_PIctrl_struct)
    addStructVarUtil("omegaPI", stru2Dtab_PIctrl_struct)
    addStructVarUtil("CH1_IdPI", stru2Dtab_PIctrl_struct)
    addStructVarUtil("CH1_IqPI", stru2Dtab_PIctrl_struct)

    addStructVarUtil("CH1_ifbk", stru2Dtab_Trans_struct)



def addStructVarUtil(structName, struInfoTab):
    global varDict
    for varInStru in struInfoTab:
        varDict[structName+"."+varInStru[0]] = varDict[structName]+varInStru[1]


class mainWindow(QtWidgets.QMainWindow, Ui_mainWIn.Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.actioncdb_GUI.triggered.connect(functools.partial(
            QtWidgets.QMessageBox.aboutQt, None, "about cdb_GUI"))

        self.statusBar().showMessage(VERSION_STR, 2000)

        # paraSet 界面部分
        # 补全器
        self.completer = QtWidgets.QCompleter(list(varDict.keys()))
        self.completer.setFilterMode(QtCore.Qt.MatchContains)
        self.completer.setCaseSensitivity(QtCore.Qt.CaseInsensitive)

        self.__paraTabInitUtil(self.lineEdit, self.lineEditV,
                               self.pushButtonR, self.pushButtonW)
        self.__paraTabInitUtil(self.lineEdit_2, self.lineEditV_2,
                               self.pushButtonR_2, self.pushButtonW_2)
        self.__paraTabInitUtil(self.lineEdit_3, self.lineEditV_3,
                               self.pushButtonR_3, self.pushButtonW_3)
        self.__paraTabInitUtil(self.lineEdit_4, self.lineEditV_4,
                               self.pushButtonR_4, self.pushButtonW_4)
        self.__paraTabInitUtil(self.lineEdit_5, self.lineEditV_5,
                               self.pushButtonR_5, self.pushButtonW_5)
        self.__paraTabInitUtil(self.lineEdit_6, self.lineEditV_6,
                               self.pushButtonR_6, self.pushButtonW_6)
        self.__paraTabInitUtil(self.lineEdit_7, self.lineEditV_7,
                               self.pushButtonR_7, self.pushButtonW_7)
        self.__paraTabInitUtil(self.lineEdit_8, self.lineEditV_8,
                               self.pushButtonR_8, self.pushButtonW_8)
        self.__paraTabInitUtil(self.lineEdit_9, self.lineEditV_9,
                               self.pushButtonR_9, self.pushButtonW_9)

        self.lineEdit.setText(paraSetPreDefName[0])
        self.lineEdit_2.setText(paraSetPreDefName[1])
        self.lineEdit_3.setText(paraSetPreDefName[2])
        self.lineEdit_4.setText(paraSetPreDefName[3])
        self.lineEdit_5.setText(paraSetPreDefName[4])
        self.lineEdit_6.setText(paraSetPreDefName[5])
        self.lineEdit_7.setText(paraSetPreDefName[6])
        self.lineEdit_8.setText(paraSetPreDefName[7])
        self.lineEdit_9.setText(paraSetPreDefName[8])

        # paraSet 界面部分
        self.lineEdit_S.setText(trigSrcPreDefName[0])
        self.lineEdit_S_2.setText(trigSrcPreDefName[1])
        self.lineEdit_S.setCompleter(self.completer)
        self.lineEdit_S_2.setCompleter(self.completer)

        self.lineEdit_L.setText(logSrcPreDefName[0])
        self.lineEdit_L_2.setText(logSrcPreDefName[1])
        self.lineEdit_L_3.setText(logSrcPreDefName[2])
        self.lineEdit_L_4.setText(logSrcPreDefName[3])
        self.lineEdit_L_5.setText(logSrcPreDefName[4])
        self.lineEdit_L_6.setText(logSrcPreDefName[5])
        self.lineEdit_L_7.setText(logSrcPreDefName[6])
        self.lineEdit_L_8.setText(logSrcPreDefName[7])
        self.lineEdit_L_9.setText(logSrcPreDefName[8])
        self.lineEdit_L_10.setText(logSrcPreDefName[9])
        self.lineEdit_L_11.setText(logSrcPreDefName[10])
        self.lineEdit_L_12.setText(logSrcPreDefName[11])
        self.lineEdit_L_13.setText(logSrcPreDefName[12])
        self.lineEdit_L_14.setText(logSrcPreDefName[13])
        self.lineEdit_L_15.setText(logSrcPreDefName[14])
        self.lineEdit_L_16.setText(logSrcPreDefName[15])
        self.lineEdit_L.setCompleter(self.completer)
        self.lineEdit_L_2.setCompleter(self.completer)
        self.lineEdit_L_3.setCompleter(self.completer)
        self.lineEdit_L_4.setCompleter(self.completer)
        self.lineEdit_L_5.setCompleter(self.completer)
        self.lineEdit_L_6.setCompleter(self.completer)
        self.lineEdit_L_7.setCompleter(self.completer)
        self.lineEdit_L_8.setCompleter(self.completer)
        self.lineEdit_L_9.setCompleter(self.completer)
        self.lineEdit_L_10.setCompleter(self.completer)
        self.lineEdit_L_11.setCompleter(self.completer)
        self.lineEdit_L_12.setCompleter(self.completer)
        self.lineEdit_L_13.setCompleter(self.completer)
        self.lineEdit_L_14.setCompleter(self.completer)
        self.lineEdit_L_15.setCompleter(self.completer)
        self.lineEdit_L_16.setCompleter(self.completer)

        self.pushButton_np.clicked.connect(self.wait4TrigSlot)
        self.pushButton_trigCfg.clicked.connect(self.trigCfgSlot)

        # dump 界面部分

        self.gridLayout_draw = QtWidgets.QGridLayout(self.groupBox_draw)
        self.gridLayout_draw.setObjectName("gridLayout_draw")

        self.drawCheckBoxList = []

        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)

        for ii in range(4):
            for jj in range(4):
                temp = QtWidgets.QCheckBox(self.groupBox_draw)
                temp.setObjectName("checkBox_"+str(ii*4+jj))
                temp.setText("checkBox_"+str(ii*4+jj))
                self.gridLayout_draw.addWidget(temp, ii, jj, 1, 1)
                temp.setSizePolicy(sizePolicy)
                self.drawCheckBoxList.append(temp)

        self.pushButton_dCfg.clicked.connect(self.dumpCfgSlot)
        self.pushButton_dump.clicked.connect(self.dumpSlot)
        self.pushButton_cb.clicked.connect(self.toCbSlot)
        self.pushButton_setAll.clicked.connect(self.setAllSlot)
        self.pushButton_clearAll.clicked.connect(self.clrAllSlot)
        self.pushButton_draw.clicked.connect(self.drawSlot)

        # exec 界面部分
        self.pushButton_clr.clicked.connect(self.plainTextEdit.clear)
        self.pushButton_exe.clicked.connect(self.execSlot)

    def execSlot(self):
        try:
            exec(self.plainTextEdit.toPlainText())
        except Exception as e:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f] exec err: ') + e.__str__())
            self.statusBar().showMessage("exec err: " + e.__str__(), 1000)
            return

    def dumpCfgSlot(self):
        global startAddr
        global dumpSize
        global endAddr
        global totalCycle
        global logTarCnt

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_sAddr.text(), None, loc)
            startAddr = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" startAddr err!")
            self.statusBar().showMessage("startAddr err!", 1000)
            return

        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_dSize.text(), None, loc)
            dumpSize = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" dumpSize err!")
            self.statusBar().showMessage("dumpSize err!", 1000)
            return

        endAddr = startAddr+2*dumpSize
        totalCycle = int(dumpSize/logTarCnt)

        global recvDict
        recvDict = {}

        global logTarNameList
        for ii in range(16):
            self.drawCheckBoxList[ii].setText(str(ii)+"-N/A")

        for ii in range(logTarCnt):
            if (logTarNameList[ii] != ""):
                self.drawCheckBoxList[ii].setText(
                    str(ii)+"-"+logTarNameList[ii])
            else:
                break

        print(datetime.datetime.now().strftime(
            '[%H:%M:%S.%f]')+" dump cfg ok!")
        self.statusBar().showMessage("dump cfg ok!", 1000)

    def dumpSlot(self):
        dumpBuff()

        global totalCycle
        global logTarCnt
        global logTarNameList
        global valYnpTab

        valYnpTab = []

        for ii in range(logTarCnt):
            valYnpTab.append(np.zeros(totalCycle))
            valtype = varTypeDict.get(logTarNameList[ii], "float")

            if (valtype == "float"):
                for jj in range(totalCycle):
                    valYnpTab[ii][jj] = struct.unpack(
                        "<f", recvDict[(jj*logTarCnt+ii)*2])[0]
            elif (valtype == "uint32_t"):
                for jj in range(totalCycle):
                    valYnpTab[ii][jj] = struct.unpack(
                        "<I", recvDict[(jj*logTarCnt+ii)*2])[0]
            elif (valtype == "int32_t"):
                for jj in range(totalCycle):
                    valYnpTab[ii][jj] = struct.unpack(
                        "<i", recvDict[(jj*logTarCnt+ii)*2])[0]
            elif (valtype == "uint16_t"):
                for jj in range(totalCycle):
                    valYnpTab[ii][jj] = struct.unpack(
                        "<H", recvDict[(jj*logTarCnt+ii)*2][0:2])[0]
            elif (valtype == "int16_t"):
                for jj in range(totalCycle):
                    valYnpTab[ii][jj] = struct.unpack(
                        "<h", recvDict[(jj*logTarCnt+ii)*2][0:2])[0]
            else:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" decoding {} type err!".format(logTarNameList[ii]))
                self.statusBar().showMessage(
                    "decoding {} type err!".format(logTarNameList[ii]), 1000)
                break

        print(datetime.datetime.now().strftime(
            '[%H:%M:%S.%f]')+" dump ok!")
        self.statusBar().showMessage("dump ok!", 1000)

    def toCbSlot(self):
        global cbStr
        global totalCycle
        global valYnpTab
        global logTarCnt
        global timex

        timex = np.arange(totalCycle)/IsrFreq

        cbStr = ""

        for ii in range(totalCycle):
            cbStr += str(timex[ii])
            for jj in range(logTarCnt):
                cbStr += "\t"
                cbStr += str(valYnpTab[jj][ii])
            cbStr += "\n"

        clipBoard = QtWidgets.QApplication.clipboard()
        clipBoard.setText(cbStr)

    def setAllSlot(self):
        for ii in range(16):
            self.drawCheckBoxList[ii].setChecked(True)

    def clrAllSlot(self):
        for ii in range(16):
            self.drawCheckBoxList[ii].setChecked(False)

    def drawSlot(self):
        global totalCycle
        global valYnpTab
        global logTarCnt
        global timex

        timex = np.arange(totalCycle)/IsrFreq
        # 画图分析
        plt.figure()

        for ii in range(logTarCnt):
            if (self.drawCheckBoxList[ii].checkState()):
                plt.plot(timex, valYnpTab[ii], label=logTarNameList[ii])

        plt.xlabel("time:s")
        plt.ylabel("value")
        plt.title("value from dump mem")
        plt.legend()
        plt.show()

    def trigCfgSlot(self):
        global trigSrcNameList
        global trigSrcAddrList
        global varDict
        trigSrcNameList = []
        trigSrcNameList.append(self.lineEdit_S.text())
        trigSrcNameList.append(self.lineEdit_S_2.text())

        try:
            trigSrcAddrList = [varDict[trigSrcNameX]
                               for trigSrcNameX in trigSrcNameList]
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigSrc name err!")
            self.statusBar().showMessage("trigSrc name err!", 1000)
            return

        global logTarNameList
        global logTarAddrList
        logTarNameList = []
        if (self.lineEdit_L.text() != ""):
            logTarNameList.append(self.lineEdit_L.text())
        if (self.lineEdit_L_2.text() != ""):
            logTarNameList.append(self.lineEdit_L_2.text())
        if (self.lineEdit_L_3.text() != ""):
            logTarNameList.append(self.lineEdit_L_3.text())
        if (self.lineEdit_L_4.text() != ""):
            logTarNameList.append(self.lineEdit_L_4.text())
        if (self.lineEdit_L_5.text() != ""):
            logTarNameList.append(self.lineEdit_L_5.text())
        if (self.lineEdit_L_6.text() != ""):
            logTarNameList.append(self.lineEdit_L_6.text())
        if (self.lineEdit_L_7.text() != ""):
            logTarNameList.append(self.lineEdit_L_7.text())
        if (self.lineEdit_L_8.text() != ""):
            logTarNameList.append(self.lineEdit_L_8.text())
        if (self.lineEdit_L_9.text() != ""):
            logTarNameList.append(self.lineEdit_L_9.text())
        if (self.lineEdit_L_10.text() != ""):
            logTarNameList.append(self.lineEdit_L_10.text())
        if (self.lineEdit_L_11.text() != ""):
            logTarNameList.append(self.lineEdit_L_11.text())
        if (self.lineEdit_L_12.text() != ""):
            logTarNameList.append(self.lineEdit_L_12.text())
        if (self.lineEdit_L_13.text() != ""):
            logTarNameList.append(self.lineEdit_L_13.text())
        if (self.lineEdit_L_14.text() != ""):
            logTarNameList.append(self.lineEdit_L_14.text())
        if (self.lineEdit_L_15.text() != ""):
            logTarNameList.append(self.lineEdit_L_15.text())
        if (self.lineEdit_L_16.text() != ""):
            logTarNameList.append(self.lineEdit_L_16.text())

        try:
            logTarAddrList = [varDict[logTarNameX]
                              for logTarNameX in logTarNameList]
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" logSrc name err!")
            self.statusBar().showMessage("logSrc name err!", 1000)
            return

        global logTarCnt
        logTarCnt = len(logTarNameList)
        if (logTarCnt < 1):
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" logSrc not enough!")
            self.statusBar().showMessage("logSrc not enough!", 1000)
            return

        # bufferSize
        global buffSize
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_buffSize.text(), None, loc)
            buffSize = int(loc['val'])
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" buffSize err!")
            self.statusBar().showMessage("buffSize err!", 1000)
            return

        # 最大采样tick
        global maxRecTick
        maxRecTick = int(buffSize/logTarCnt)

        global logKeepRatio
        global logKeepDataNumBeforeTrig
        logKeepRatio = self.doubleSpinBox.value()
        logKeepDataNumBeforeTrig = int(logKeepRatio * maxRecTick)*logTarCnt

        # 触发时间阈值
        global trigTimeTar
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_trigtime.text(), None, loc)
            trigTimeTar = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigTimeTar err!")
            self.statusBar().showMessage("trigTimeTar err!", 1000)
            return

        # 触发阈值
        global trigThd
        try:
            loc = {"val": 0}
            exec("val = "+self.lineEdit_trigThd.text(), None, loc)
            trigThd = loc['val']
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigThd err!")
            self.statusBar().showMessage("trigThd err!", 1000)
            return

        # 源模式
        # bit 0-2 类型: 0 int16; 1 uint16; 2 int32; 3 uint32; 4 float; 5 强制0;
        # bit 3 是否取反
        # src[0]在低位 src[1]在高位
        global trigSrc
        temp1 = self.comboBox.currentIndex()
        if (self.checkBox.checkState()):
            temp1 += 8
        temp2 = self.comboBox_2.currentIndex()
        if (self.checkBox_2.checkState()):
            temp2 += 8
        trigSrc = temp1+temp2*16

        # 间隔采样
        global sampInter
        sampInter = self.spinBox.value()
        # 触发模式
        global trigMode
        trigMode = self.comboBox_trigMode.currentIndex()

        try:
            setTrigConf()
        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigCfg err!")
            self.statusBar().showMessage("trigCfg err!", 1000)
        else:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" trigCfg OK!")
            self.statusBar().showMessage("trigCfg OK!", 1000)

    def paraSetRSlot(self, nameEdit, valEdit):
        try:
            ret = readMCUVarUtil(nameEdit.text())
        except KeyError:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" var name err!")
            self.statusBar().showMessage("var name err!", 1000)
        else:
            if (ret != None):
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" paraRead:{} = {}".format(nameEdit.text(), ret))
                valEdit.setText(str(ret))
                self.statusBar().showMessage("read ok", 1000)

    def paraSetWSlot(self, nameEdit, valEdit):
        try:
            val = 0
            loc = {"val": 0}
            exec("val = "+valEdit.text()+"\n", None, loc)
            val = loc['val']

        except Exception:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" val err!")
            self.statusBar().showMessage("var err!", 1000)
        else:
            try:
                ret = setMCUVarUtil(nameEdit.text(), val)
            except KeyError:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" var name err!")
                self.statusBar().showMessage("var name err!", 1000)
            else:
                if (ret == True):
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" paraSet:{} = {}".format(nameEdit.text(), val))
                    self.statusBar().showMessage("write ok", 1000)

    def __paraTabInitUtil(self, nameEdit, valEdit, Rbtn, Wbtn):
        Rbtn.clicked.connect(functools.partial(
            self.paraSetRSlot, nameEdit, valEdit))
        Wbtn.clicked.connect(functools.partial(
            self.paraSetWSlot, nameEdit, valEdit))
        nameEdit.setCompleter(self.completer)

    def wait4TrigSlot(self):
        global waitingFlg
        if (waitingFlg):
            # 正在等就不等了
            waitingFlg = False
            self.pushButton_np.setText("wait for trig")
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" manually canceled!")
            self.statusBar().showMessage("manually canceled!", 1000)
        else:
            waitingFlg = True
            trigSta = 200
            self.pushButton_np.setText("waiting...")

            while (waitingFlg):
                QtWidgets.QApplication.processEvents()
                time.sleep(0.2)

                try:
                    trigSta = readMCUVarUtil("cdb1.trigSta")
                    pass
                except Exception:
                    waitingFlg = False
                    self.pushButton_np.setText("wait for trig")
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" CAN com err while wait!")
                    self.statusBar().showMessage("CAN com err while wait!", 1000)
                    return

                if (trigSta >= 5):
                    waitingFlg = False
                    self.pushButton_np.setText("wait for trig")
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" trig detected!")
                    self.statusBar().showMessage("trig detected!", 1000)
                    self.tabWidget.setCurrentIndex(2)
                    return


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


def readMCUVarUtil(namex):
    global ch

    if (namex in varTypeDict):
        # TODO 补充其他情况
        if (varTypeDict[namex] == "uint16_t"):

            ch = canlib.openChannel(
                channel=0,
                flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
                bitrate=canlib.Bitrate.BITRATE_500K,
            )
            ch.setBusOutputControl(canlib.Driver.NORMAL)
            ch.busOn()

            try:
                txFrameData = struct.pack(
                    "<I", varDict[namex]) + struct.pack("<I", 0)
                txFrame = Frame(id_=6, data=txFrameData, dlc=8)
                ch.write(txFrame)
                ch.writeSync(timeout=200)

                frame = ch.read(timeout=200)
                if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" frame err!")
                else:
                    can_val = struct.unpack("<H", frame.data[4:6])[0]

            except canlib.CanNoMsg:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" no reply!")
                can_val = None
            finally:
                ch.busOff()
                ch.close()

            return can_val

        else:
            # TODO 补充其他情况
            print("type not supported!")
    else:
        ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
            bitrate=canlib.Bitrate.BITRATE_500K,
        )
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        try:
            txFrameData = struct.pack(
                "<I", varDict[namex]) + struct.pack("<I", 2)
            txFrame = Frame(id_=6, data=txFrameData, dlc=8)
            ch.write(txFrame)
            ch.writeSync(timeout=200)

            frame = ch.read(timeout=200)
            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" frame err!")
            else:
                can_val = struct.unpack("<f", frame.data[4:8])[0]

        except canlib.CanNoMsg:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" no reply!")
            can_val = None

        finally:
            ch.busOff()
            ch.close()

        return can_val

# 设置变量的辅助函数


def setMCUVarUtil(namex, val):
    global ch
    comSta = True

    if (namex in varTypeDict):
        if (varTypeDict[namex] == "uint16_t"):

            ch = canlib.openChannel(
                channel=0,
                flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
                bitrate=canlib.Bitrate.BITRATE_500K,
            )
            ch.setBusOutputControl(canlib.Driver.NORMAL)
            ch.busOn()

            try:
                txFrameData = struct.pack(
                    "<I", varDict[namex]) + struct.pack("<H", val) + struct.pack("<H", 0)
                txFrame = Frame(id_=3, data=txFrameData, dlc=8)
                ch.write(txFrame)
                ch.writeSync(timeout=200)

                frame = ch.read(timeout=200)
                if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" frame err!")
                else:
                    can_val = struct.unpack("<H", frame.data[4:6])[0]
                    if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                        print(datetime.datetime.now().strftime(
                            '[%H:%M:%S.%f]')+" CAN err!")
                        comSta = False

            except canlib.CanNoMsg:
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" no reply!")
                comSta = False

            finally:
                ch.busOff()
                ch.close()

            return comSta

        else:
            # TODO 补充其他情况
            print("type not supported!")
    else:
        ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
            bitrate=canlib.Bitrate.BITRATE_500K,
        )
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()

        try:
            txFrameData = struct.pack(
                "<I", varDict[namex]) + struct.pack("<f", val)
            txFrame = Frame(id_=4, data=txFrameData, dlc=8)
            ch.write(txFrame)
            ch.writeSync(timeout=200)

            frame = ch.read(timeout=200)
            if (frame.flags != 2 or frame.id != 5 or frame.dlc != 8):
                print(datetime.datetime.now().strftime(
                    '[%H:%M:%S.%f]')+" frame err!")
            else:
                can_val = struct.unpack("<f", frame.data[4:8])[0]
                if (abs(val-can_val)/(abs(val)+1e-12) > 0.01):
                    print(datetime.datetime.now().strftime(
                        '[%H:%M:%S.%f]')+" CAN err!")
                    comSta = False

        except canlib.CanNoMsg:
            print(datetime.datetime.now().strftime(
                '[%H:%M:%S.%f]')+" no reply!")
            comSta = False

        finally:
            ch.busOff()
            ch.close()

        return comSta

# 设置触发的辅助函数


def setTrigConf():
    # 通过can写入触发配置
    global ch
    global trigSrcNameList
    global trigSrcAddrList
    global logTarNameList
    global logTarAddrList
    global logTarCnt
    global buffSize
    global maxRecTick
    global logKeepDataNumBeforeTrig
    global trigTimeTar
    global trigThd
    global trigSrc
    global sampInter
    global trigMode

    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=canlib.Bitrate.BITRATE_500K,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    try:
        # 源地址
        for ii in range(len(trigSrcAddrList)):
            txFrameData = struct.pack("<I", ii) + \
                struct.pack("<I", trigSrcAddrList[ii])
            txFrame = Frame(id_=2, data=txFrameData, dlc=8)
            ch.write(txFrame)
            QtWidgets.QApplication.processEvents()
            ch.writeSync(timeout=200)
            time.sleep(0.1)

        # 记录项
        for ii in range(len(logTarAddrList)):
            txFrameData = struct.pack("<I", ii + 2) + \
                struct.pack("<I", logTarAddrList[ii])
            txFrame = Frame(id_=2, data=txFrameData, dlc=8)
            ch.write(txFrame)
            QtWidgets.QApplication.processEvents()
            ch.writeSync(timeout=200)
            time.sleep(0.1)

        # 需记录的数据个数 18
        txFrameData = struct.pack("<I", 18) + struct.pack("<I", logTarCnt)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 需保存的触发前的数据个数 19
        txFrameData = struct.pack("<I", 19) + \
            struct.pack("<I", logKeepDataNumBeforeTrig)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 和时间有关触发的目标值 20
        txFrameData = struct.pack("<I", 20) + struct.pack("<I", trigTimeTar)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发阈值 21
        txFrameData = struct.pack("<I", 21) + struct.pack("<f", trigThd)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发值选择 22
        txFrameData = struct.pack("<I", 22) + struct.pack("<I", trigSrc)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 采样间隔 23
        txFrameData = struct.pack("<I", 23) + struct.pack("<I", sampInter)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)
        # 触发模式 24
        txFrameData = struct.pack("<I", 24) + struct.pack("<I", trigMode)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)
        time.sleep(0.1)

        # 触发器状态 25
        txFrameData = struct.pack("<I", 25) + struct.pack("<I", 1)
        txFrame = Frame(id_=2, data=txFrameData, dlc=8)
        ch.write(txFrame)
        QtWidgets.QApplication.processEvents()
        ch.writeSync(timeout=200)

    finally:
        ch.busOff()
        ch.close()

# dump RAM的辅助函数


def dumpBuff():
    global ch
    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=canlib.Bitrate.BITRATE_500K,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()

    global startAddr
    global endAddr
    txFrameData = struct.pack("<I", startAddr) + struct.pack("<I", endAddr)
    txFrame = Frame(id_=1, data=txFrameData, dlc=8)

    try:
        ch.write(txFrame)
    except Exception:
        ch.busOff()
        ch.close()
        print(datetime.datetime.now().strftime(
            '\n[%H:%M:%S.%f]')+" dump com fail!")
        self.statusBar().showMessage("dump com fail!", 1000)
        return

    global recvDict

    # 监听端口
    timeout = 0.2
    ticktime = 2
    tick_countup = 0
    frame_cnt = 0
    frame_cnt2 = 0
    frame_err = 0

    while True:
        try:
            frame = ch.read(timeout=int(timeout * 1000))
            # 最基本的校验
            if (frame.flags != 2 or frame.id < 8 or frame.id > 15 or frame.dlc != 8):
                frame_err += 1
                print("err cnt: {}\n".format(frame_err))
                w.statusBar().showMessage("err cnt: {}".format(frame_err), 1000)
                continue

            frame_cnt += 1
            frame_cnt2 += 1
            if (frame_cnt2 == 100):
                print("\rnow cnt: {}".format(frame_cnt), end="")
                w.statusBar().showMessage("now cnt: {}".format(frame_cnt), 1000)
                QtWidgets.QApplication.processEvents()
                frame_cnt2 = 0

            can_addr = struct.unpack("<I", frame.data[0:4])[0]

            # raw
            can_val = frame.data[4:8]
            recvDict[can_addr-startAddr] = can_val

        except canlib.CanNoMsg:
            tick_countup += timeout

            if (frame_cnt == dumpSize):
                print(datetime.datetime.now().strftime(
                    '\r[%H:%M:%S.%f]')+" recv ok! cnt: {}".format(frame_cnt))
                w.statusBar().showMessage("recv ok! cnt: {}".format(frame_cnt), 1000)
                break

            if (frame_cnt > dumpSize):
                print(datetime.datetime.now().strftime(
                    '\n[%H:%M:%S.%f]')+" recved {} pkgs, too more!".format(frame_cnt))
                w.statusBar().showMessage("recved {} pkgs, too more!".format(frame_cnt), 1000)
                break

            if tick_countup > ticktime:
                print(datetime.datetime.now().strftime(
                    '\n[%H:%M:%S.%f]')+" dump time out!")
                w.statusBar().showMessage("dump time out!", 1000)
                break

        except Exception:
            print(datetime.datetime.now().strftime(
                '\n[%H:%M:%S.%f]')+" other dump err!")
            w.statusBar().showMessage("other dump err!", 1000)
            ch.busOff()
            ch.close()
            return

    ch.busOff()
    ch.close()


if __name__ == "__main__":
    print(
        "\n"+datetime.datetime.now().strftime('[%H:%M:%S.%f] ')+VERSION_STR+" sdardle!\n")
    # 枚举端口
    enumPorts()

    # 建立变量字典
    varDict = {}
    varTypeDict = {}
    setupVarDict()

    # 尝试打开端口
    # 保证变量全局性？
    ch = canlib.openChannel(
        channel=0,
        flags=canlib.Open.EXCLUSIVE | canlib.Open.ACCEPT_VIRTUAL,
        bitrate=canlib.Bitrate.BITRATE_500K,
    )
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.busOn()
    ch.busOff()
    ch.close()

    # 建立触发配置的数据结构
    trigSrcNameList = ["a"]
    trigSrcAddrList = []
    logTarNameList = ["a"]
    logTarAddrList = []
    logTarCnt = 1
    buffSize = int(15 * 4 * 1024 / 2)
    maxRecTick = int(buffSize/logTarCnt)
    logKeepRatio = 0.2
    logKeepDataNumBeforeTrig = int(logKeepRatio * maxRecTick)*logTarCnt
    trigTimeTar = 40 * 2
    trigThd = 280
    trigSrc = 0b0101_0100
    sampInter = 0
    trigMode = 0

    # 建立dump的数据结构
    startAddr = 0x9000
    dumpSize = buffSize
    endAddr = startAddr+2*dumpSize
    totalCycle = maxRecTick
    cbStr = "Str 2 cb test"
    recvDict = {}
    valYnpTab = []
    timex = np.arange(totalCycle)/IsrFreq

    # 等待标记
    waitingFlg = False

    app = QtWidgets.QApplication(sys.argv)
    fontx = QtGui.QFont()
    fontx.setPixelSize(14)
    app.setFont(fontx)

    w = mainWindow()
    w.show()

    sys.exit(app.exec())
