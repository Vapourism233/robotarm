from imreadpic import *
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5 import QtGui,QtCore
from Threshold_Myself import threshold_self
from replace import *
from Find_Row_and_Column import *
from fbs_runtime.application_context.PyQt5 import ApplicationContext

class myWin(QMainWindow,Ui_Dialog):
    def __init__(self,parent=None):
        super(myWin,self).__init__(parent)
        self.setupUi(self)
        self.pushButton.clicked.connect(self.openimage)
        self.pushButton_2.clicked.connect(self.devide_file)
        self.pushButton_3.clicked.connect(self.openTemplate)
        self.pushButton_4.clicked.connect(self.Find_Row_and_Column)
        self.pushButton_5.clicked.connect(self.replace_old_template)

    def openimage(self):
        imgName, imgType = QFileDialog.getOpenFileName(self,"打开图片","", "*.jpg;;*.bmp;;*.png;;All Files (*)")
        jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
        self.label.setPixmap(jpg)
        self.lineEdit.setText(imgName)
    def openTemplate(self):
        imgName, imgType = QFileDialog.getOpenFileName(self,"打开模板","", "*.jpg;;*.bmp;;*.png;;All Files (*)")

        self.lineEdit_3.setText(imgName)

    def devide_file(self):
        Value = self.lineEdit_2.text()          #注意区分lineEdit和textEdit的区别，读取文本的代码不同
        file_path = self.lineEdit.text()
        template_path = self.lineEdit_3.text()
        if file_path == '':
            self.showMessageBox()
        else:
            if template_path == '':
                self.showMessageBox2()
            else:
                threshold_self.threshold_image(file_path,Value)

    def replace_old_template(self):
        file_path = self.lineEdit.text()
        template_path = self.lineEdit_3.text()
        if file_path == '':
            self.showMessageBox()
        else:
            if template_path == '':
                self.showMessageBox2()
            else:
                Replace_Template.exercute(file_path,template_path)



    def closeEvent(self,event):                 #PyQt5自带，重新改写关闭窗口函数，并继承主窗口的所有控件
        reply = QtWidgets.QMessageBox.question(self,
                                               '提示', "你确定要退出吗？",
                                               QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                               QtWidgets.QMessageBox.No)            #注意格式
        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

    def Find_Row_and_Column(self):
        file_path = self.lineEdit.text()
        template_path = self.lineEdit_3.text()
        if file_path == '':
            self.showMessageBox()
        else:
            if template_path == '':
                self.showMessageBox2()
            else:
                Find_Row_and_Column.Find_R_C(file_path,template_path)
    def showMessageBox(self):
        res = QtWidgets.QMessageBox.warning(self,'警告','请选择文件后再进行操作！',QtWidgets.QMessageBox.Yes)

    def showMessageBox2(self):
        res1 = QtWidgets.QMessageBox.warning(self,'警告','请选择模板后再进行操作！',QtWidgets.QMessageBox.Yes)
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mywin = myWin()
    mywin.show()
    sys.exit(app.exec_())