#!/usr/bin/env python
from __future__ import print_function

from click import command
import rospy, rospkg
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg
import pacmod_msgs.msg as pacmod_m
import std_msgs.msg as stdmsg
import numpy as np
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea


class PlotHandler(object):
    def __init__(self, vehicle):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.vehicle = vehicle
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()
        self.rospack = rospkg.RosPack()
        self.win.setWindowFlags(qtgqt.QtCore.Qt.WindowStaysOnTopHint)

    def initializePlot(self):
        self.first_run = True
        white = (200, 200, 200)
        lightgray = (171, 178, 191)
        darkgray = (30, 40, 50)
        dark1 = (40, 44, 52)
        dark2 = (44, 48, 56)
        red = (200, 66, 66); red1b = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blue1b = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); green1b = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellow1b = pg.mkBrush(244, 244, 160, 200)
             
        self.win.setWindowTitle("km/h")
        self.win.setWindowIcon(qtgqt.QtGui.QIcon(self.rospack.get_path("lexus_base") + "/img/icon05.png"))
        self.win.resize(680,30)
        self.win.setCentralWidget(self.area)

        self.dleftbottom = darea.Dock("left bottom", size = (500,400)) # size is only a suggestion
        self.dleftbottom.hideTitleBar() # TODO 1
        self.area.addDock(self.dleftbottom) # , "bottom", self.dlefttop
        self.speedLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.speedLabel.setStyleSheet("font-family: Monospace; font: 30pt; background-color: rgb(0, 0, 0)")
        self.state = None
        self.widgplot = pg.PlotWidget() # TODO 1
        self.widgplot.setBackground('w')
        self.widgplot.setAspectLocked(True)
        self.plot_left2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(6,106,166,255))
        self.widgplot.showGrid(x=True, y=True)
        self.widgplot.addItem(self.plot_left2)
        self.dleftbottom.addWidget(self.widgplot)
        self.widgplot.hideAxis('bottom') # TODO 1
        self.widgplot.hideAxis('left') # TODO 1
        self.tcurr = pg.TextItem(text="Vehicle status", color = white)
        self.tstart = pg.TextItem(text="Start", color = red)
        self.redSpeedText1 = pg.TextItem(text="-", color = red) 
        self.widgplot.addItem(self.redSpeedText1)
        self.statusText1 = pg.TextItem(text="-", color = red, anchor=(0.5,0))
        self.statusText1.setPos(0, 180)
        self.statusText1.setFont(qtgqt.QtGui.QFont('Sans Bold', 18, qtgqt.QtGui.QFont.Bold))
        self.redSpeedText1.setFont(qtgqt.QtGui.QFont('Sans Bold', 20, qtgqt.QtGui.QFont.Bold))
        self.win.show()
        self.win.move(10,20) # TODO 2
        self.widgplot.setXRange(-300, 400, padding=0) # TODO 2
        self.widgplot.setYRange(-60, 20, padding=0) # TODO 2
        self.blinker = True

    def updateSlow(self):
        None

    def updateFast(self):
        self.redSpeedText1.setText("%.0f km/h" % self.vehicle.actual_speed)
        self.redSpeedText1.setPos(0, 0)


    def updateLabels(self):
        self.speedLabel.setText("actual: %4.1f km/h\nrefer : %4.1f km/h" % (self.vehicle.actual_speed, self.vehicle.reference_speed))        
        #self.isAutonomLabel.setText("x: %9.6f\ny: %9.6f" % (self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y))          



class VehicleStatusSub(object):
    def __init__(self):
        rospy.Subscriber("/pacmod/as_tx/vehicle_speed", stdmsg.Float64, self.speedKmpHCallBack)
        rospy.Subscriber("/pacmod/as_tx/enabled", stdmsg.Bool, self.vehicleStatusCallback)

        self.actual_speed = -1
        self.reference_speed = -1
        self.leaf_is_autonomous = "UNDEF"

    def speedKmpHCallBack(self, msg):
        self.actual_speed = np.array([msg.data]) * 3.6

    def vehicleStatusCallback(self, msg):
        if msg.data == False: 
            self.leaf_is_autonomous = "IN CAR DRIVER"
        elif msg.data == True:
            self.leaf_is_autonomous = "GYOR"
        else:
            self.leaf_is_autonomous = "UNDEF"

if __name__ == "__main__":
    import sys
    rospy.init_node("plotter_kmph_", anonymous=True)
    rospy.loginfo("Vehicle km/h status GUI started ... ")
    vehSub = VehicleStatusSub()
    ph = PlotHandler(vehSub)
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateFast)
    timer1.start(30)
    timer2 = qtgqt.QtCore.QTimer()
    timer2.timeout.connect(ph.updateSlow)
    timer2.start(600)
    timer3 = qtgqt.QtCore.QTimer()
    timer3.timeout.connect(ph.updateLabels)
    timer3.start(30)


    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()