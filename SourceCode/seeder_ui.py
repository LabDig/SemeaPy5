# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'seeder_ui.ui'
#
# Created: Thu Mar 22 00:51:32 2018
#      by: PyQt5 UI code generator 5.3.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_TabWidget(object):
    def setupUi(self, TabWidget):
        TabWidget.setObjectName("TabWidget")
        TabWidget.resize(800, 430)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("/home/debian/semeadora_qt5/icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        TabWidget.setWindowIcon(icon)
        self.main = QtWidgets.QWidget()
        self.main.setObjectName("main")
        self.carrega = QtWidgets.QPushButton(self.main)
        self.carrega.setGeometry(QtCore.QRect(400, 0, 371, 70))
        self.carrega.setMinimumSize(QtCore.QSize(0, 70))
        self.carrega.setMaximumSize(QtCore.QSize(16777215, 70))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.carrega.setFont(font)
        self.carrega.setObjectName("carrega")
        self.vel_3 = QtWidgets.QComboBox(self.main)
        self.vel_3.setGeometry(QtCore.QRect(910, 120, 91, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.vel_3.setFont(font)
        self.vel_3.setObjectName("vel_3")
        self.vel_3.addItem("")
        self.vel_3.addItem("")
        self.vel_3.addItem("")
        self.vel_3.addItem("")
        self.layoutWidget = QtWidgets.QWidget(self.main)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 320, 251, 32))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_10 = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_5.addWidget(self.label_10)
        self.st_semeadora = QtWidgets.QPlainTextEdit(self.layoutWidget)
        self.st_semeadora.setMaximumSize(QtCore.QSize(150, 30))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.st_semeadora.setFont(font)
        self.st_semeadora.setObjectName("st_semeadora")
        self.horizontalLayout_5.addWidget(self.st_semeadora)
        self.layoutWidget1 = QtWidgets.QWidget(self.main)
        self.layoutWidget1.setGeometry(QtCore.QRect(280, 320, 261, 32))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_14 = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_6.addWidget(self.label_14)
        self.vel_sensor = QtWidgets.QPlainTextEdit(self.layoutWidget1)
        self.vel_sensor.setMaximumSize(QtCore.QSize(100, 30))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.vel_sensor.setFont(font)
        self.vel_sensor.setObjectName("vel_sensor")
        self.horizontalLayout_6.addWidget(self.vel_sensor)
        self.layoutWidget2 = QtWidgets.QWidget(self.main)
        self.layoutWidget2.setGeometry(QtCore.QRect(550, 320, 131, 32))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_11 = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_7.addWidget(self.label_11)
        self.pdop = QtWidgets.QPlainTextEdit(self.layoutWidget2)
        self.pdop.setMaximumSize(QtCore.QSize(80, 30))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pdop.setFont(font)
        self.pdop.setObjectName("pdop")
        self.horizontalLayout_7.addWidget(self.pdop)
        self.layoutWidget3 = QtWidgets.QWidget(self.main)
        self.layoutWidget3.setGeometry(QtCore.QRect(480, 360, 201, 32))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.layoutWidget3)
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_28 = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_28.setFont(font)
        self.label_28.setObjectName("label_28")
        self.horizontalLayout_9.addWidget(self.label_28)
        self.line_plantas = QtWidgets.QPlainTextEdit(self.layoutWidget3)
        self.line_plantas.setMaximumSize(QtCore.QSize(80, 30))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.line_plantas.setFont(font)
        self.line_plantas.setObjectName("line_plantas")
        self.horizontalLayout_9.addWidget(self.line_plantas)
        self.layoutWidget4 = QtWidgets.QWidget(self.main)
        self.layoutWidget4.setGeometry(QtCore.QRect(10, 360, 451, 32))
        self.layoutWidget4.setObjectName("layoutWidget4")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.layoutWidget4)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_27 = QtWidgets.QLabel(self.layoutWidget4)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_27.setFont(font)
        self.label_27.setObjectName("label_27")
        self.horizontalLayout_8.addWidget(self.label_27)
        self.line_rot = QtWidgets.QPlainTextEdit(self.layoutWidget4)
        self.line_rot.setMaximumSize(QtCore.QSize(180, 30))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.line_rot.setFont(font)
        self.line_rot.setObjectName("line_rot")
        self.horizontalLayout_8.addWidget(self.line_rot)
        self.gv = QtWidgets.QGraphicsView(self.main)
        self.gv.setGeometry(QtCore.QRect(400, 70, 371, 241))
        self.gv.setObjectName("gv")
        self.layoutWidget5 = QtWidgets.QWidget(self.main)
        self.layoutWidget5.setGeometry(QtCore.QRect(10, 10, 375, 281))
        self.layoutWidget5.setObjectName("layoutWidget5")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.layoutWidget5)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_29 = QtWidgets.QLabel(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_29.setFont(font)
        self.label_29.setObjectName("label_29")
        self.horizontalLayout_13.addWidget(self.label_29)
        self.modo = QtWidgets.QComboBox(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.modo.setFont(font)
        self.modo.setObjectName("modo")
        self.modo.addItem("")
        self.modo.addItem("")
        self.horizontalLayout_13.addWidget(self.modo)
        self.verticalLayout_2.addLayout(self.horizontalLayout_13)
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_12 = QtWidgets.QLabel(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_14.addWidget(self.label_12)
        self.espac = QtWidgets.QComboBox(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.espac.setFont(font)
        self.espac.setObjectName("espac")
        self.espac.addItem("")
        self.espac.addItem("")
        self.espac.addItem("")
        self.espac.addItem("")
        self.espac.addItem("")
        self.espac.addItem("")
        self.horizontalLayout_14.addWidget(self.espac)
        self.verticalLayout_2.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_16 = QtWidgets.QLabel(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_16.setFont(font)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_15.addWidget(self.label_16)
        self.pol = QtWidgets.QComboBox(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.pol.setFont(font)
        self.pol.setObjectName("pol")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.pol.addItem("")
        self.horizontalLayout_15.addWidget(self.pol)
        self.verticalLayout_2.addLayout(self.horizontalLayout_15)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_19 = QtWidgets.QLabel(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_19.setFont(font)
        self.label_19.setObjectName("label_19")
        self.horizontalLayout.addWidget(self.label_19)
        self.germ = QtWidgets.QSpinBox(self.layoutWidget5)
        self.germ.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.germ.setFont(font)
        self.germ.setMinimum(50)
        self.germ.setMaximum(100)
        self.germ.setProperty("value", 85)
        self.germ.setObjectName("germ")
        self.horizontalLayout.addWidget(self.germ)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_9 = QtWidgets.QLabel(self.layoutWidget5)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_3.addWidget(self.label_9)
        self.furos = QtWidgets.QComboBox(self.layoutWidget5)
        self.furos.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.furos.setFont(font)
        self.furos.setObjectName("furos")
        self.furos.addItem("")
        self.furos.addItem("")
        self.furos.addItem("")
        self.furos.addItem("")
        self.horizontalLayout_3.addWidget(self.furos)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
        self.sair = QtWidgets.QPushButton(self.main)
        self.sair.setGeometry(QtCore.QRect(690, 320, 111, 70))
        self.sair.setMinimumSize(QtCore.QSize(0, 70))
        self.sair.setMaximumSize(QtCore.QSize(16777215, 70))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.sair.setFont(font)
        self.sair.setObjectName("sair")
        TabWidget.addTab(self.main, "")

        self.retranslateUi(TabWidget)
        TabWidget.setCurrentIndex(0)
        self.sair.clicked.connect(TabWidget.close)
        QtCore.QMetaObject.connectSlotsByName(TabWidget)

    def retranslateUi(self, TabWidget):
        _translate = QtCore.QCoreApplication.translate
        TabWidget.setWindowTitle(_translate("TabWidget", "Seeder"))
        self.carrega.setText(_translate("TabWidget", "LOAD MAP"))
        self.vel_3.setItemText(0, _translate("TabWidget", "0,75"))
        self.vel_3.setItemText(1, _translate("TabWidget", "1,0"))
        self.vel_3.setItemText(2, _translate("TabWidget", "1,25"))
        self.vel_3.setItemText(3, _translate("TabWidget", "1,5"))
        self.label_10.setText(_translate("TabWidget", "STATUS:"))
        self.label_14.setText(_translate("TabWidget", "VELOCITY (M/S)"))
        self.label_11.setText(_translate("TabWidget", " PDOP:"))
        self.label_28.setText(_translate("TabWidget", "SEED/M :"))
        self.label_27.setText(_translate("TabWidget", "MOTOR VELOCITY (RPM)"))
        self.label_29.setText(_translate("TabWidget", "MODE"))
        self.modo.setItemText(0, _translate("TabWidget", "MANUAL"))
        self.modo.setItemText(1, _translate("TabWidget", "MAP"))
        self.label_12.setText(_translate("TabWidget", "SPACING (M):"))
        self.espac.setItemText(0, _translate("TabWidget", "0.7"))
        self.espac.setItemText(1, _translate("TabWidget", "0.8"))
        self.espac.setItemText(2, _translate("TabWidget", "1.0"))
        self.espac.setItemText(3, _translate("TabWidget", "0.4"))
        self.espac.setItemText(4, _translate("TabWidget", "0.5"))
        self.espac.setItemText(5, _translate("TabWidget", "0.6"))
        self.label_16.setText(_translate("TabWidget", "POPULATION(PLANTS/HA):"))
        self.pol.setItemText(0, _translate("TabWidget", "70000"))
        self.pol.setItemText(1, _translate("TabWidget", "60000"))
        self.pol.setItemText(2, _translate("TabWidget", "50000"))
        self.pol.setItemText(3, _translate("TabWidget", "40000"))
        self.pol.setItemText(4, _translate("TabWidget", "225000"))
        self.pol.setItemText(5, _translate("TabWidget", "30000"))
        self.pol.setItemText(6, _translate("TabWidget", "80000"))
        self.pol.setItemText(7, _translate("TabWidget", "100000"))
        self.pol.setItemText(8, _translate("TabWidget", "200000"))
        self.pol.setItemText(9, _translate("TabWidget", "250000"))
        self.pol.setItemText(10, _translate("TabWidget", "300000"))
        self.label_19.setText(_translate("TabWidget", "GERMINATION (%)"))
        self.label_9.setText(_translate("TabWidget", "# OF HOLES"))
        self.furos.setItemText(0, _translate("TabWidget", "28"))
        self.furos.setItemText(1, _translate("TabWidget", "45"))
        self.furos.setItemText(2, _translate("TabWidget", "64"))
        self.furos.setItemText(3, _translate("TabWidget", "84"))
        self.sair.setText(_translate("TabWidget", "EXIT"))
        TabWidget.setTabText(TabWidget.indexOf(self.main), _translate("TabWidget", "MAIN"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    TabWidget = QtWidgets.QTabWidget()
    ui = Ui_TabWidget()
    ui.setupUi(TabWidget)
    TabWidget.show()
    sys.exit(app.exec_())

