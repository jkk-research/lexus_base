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
             
        self.win.setWindowTitle("Vehicle status plotter")
        self.win.setWindowIcon(qtgqt.QtGui.QIcon(self.rospack.get_path("lexus_base") + "/img/icon02.png"))
        self.win.resize(300,300)
        self.win.setCentralWidget(self.area)

        self.dleftbottom = darea.Dock("left bottom", size = (500,400)) # size is only a suggestion
        #self.dlefttop.hideTitleBar() # TODO 1
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
        self.blueWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=8))
        self.widgplot.addItem(self.blueWheelHoriz)
        self.blueWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=8))
        self.widgplot.addItem(self.blueWheelVertic)
        self.blueWheelText1 = pg.TextItem(text="-", color = blue)
        self.widgplot.addItem(self.blueWheelText1)
        self.drawBlueCircle(self.widgplot)

        #self.blueSpeedBar = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=8))
        #self.widgplot.addItem(self.blueSpeedBar)

        self.drawRedCircle(self.widgplot)
        self.redWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        self.redWheelText1 = pg.TextItem(text="-", color = red)
        self.widgplot.addItem(self.redWheelHoriz)
        self.widgplot.addItem(self.redWheelText1)
        self.redSpeedBar = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        self.widgplot.addItem(self.redSpeedBar)
        self.redSpeedText1 = pg.TextItem(text="-", color = red)
        self.widgplot.addItem(self.redSpeedText1)
        #self.blueSpeedText1 = pg.TextItem(text="-", color = blue)
        #self.widgplot.addItem(self.blueSpeedText1)
        self.statusText1 = pg.TextItem(text="-", color = red, anchor=(0.5,0))
        self.widgplot.addItem(self.statusText1)  
        self.statusText1.setPos(0, 180)
        self.statusText1.setFont(qtgqt.QtGui.QFont('Sans Bold', 18, qtgqt.QtGui.QFont.Bold))

        self.redWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        self.widgplot.addItem(self.redWheelVertic)


        #self.widgplot.setAspectLocked(True)
        self.win.show()
        self.win.move(1020,20) # TODO 2
        self.widgplot.setXRange(-150, 150, padding=0) # TODO 2
        self.widgplot.setYRange(-120, 180, padding=0) # TODO 2
        self.blinker = True

    def updateSlow(self):
        self.statusText1.setText(str(self.vehicle.leaf_is_autonomous))
        if str(self.vehicle.leaf_is_autonomous) == "UNDEF":
            self.blinker = not self.blinker
            if self.blinker:
                self.statusText1.setColor(qtgqt.QtGui.QColor(0, 40, 100))
            else:
                self.statusText1.setColor(qtgqt.QtGui.QColor(244, 244, 160))
        elif str(self.vehicle.leaf_is_autonomous) == "IN CAR DRIVER":
            self.blinker = not self.blinker
            if self.blinker:
                self.statusText1.setColor(qtgqt.QtGui.QColor(0, 40, 100))
            else:
                self.statusText1.setColor(qtgqt.QtGui.QColor(150, 180, 240))
        elif str(self.vehicle.leaf_is_autonomous) == "YOU DRIVE":
            self.statusText1.setColor(qtgqt.QtGui.QColor(20, 140, 70))
        else:
            self.statusText1.setColor(qtgqt.QtGui.QColor(50, 60, 110))

    def updateFast(self):
        self.redWheelText1.setText("%.0f" % np.rad2deg(self.vehicle.wheel_actual_rad))
        self.blueWheelText1.setText("%.0f" % np.rad2deg(self.vehicle.wheel_reference_rad))
        self.redWheelText1.setPos(110 * np.cos(self.vehicle.wheel_actual_rad), 110 * np.sin(self.vehicle.wheel_actual_rad))
        self.blueWheelText1.setPos(-120 * np.cos(self.vehicle.wheel_reference_rad), -120 * np.sin(self.vehicle.wheel_reference_rad))
        r1 = np.array([100 * np.cos(self.vehicle.wheel_actual_rad), -100 * np.cos(self.vehicle.wheel_actual_rad)], dtype = np.float).flatten()
        r2 = np.array([100 * np.sin(self.vehicle.wheel_actual_rad), -100 * np.sin(self.vehicle.wheel_actual_rad)], dtype = np.float).flatten()
        r0 = np.array([100 * np.cos(self.vehicle.wheel_actual_rad - np.math.pi / 2), 0.], dtype = np.float)
        r3 = np.array([100 * np.sin(self.vehicle.wheel_actual_rad - np.math.pi / 2), 0.], dtype = np.float)
        xx = 20.0 # TODO  self.vehicle.actual_speed
        posr = (self.vehicle.actual_speed * 10) - 100
        posb = (self.vehicle.reference_speed * 10) - 100
        self.redSpeedText1.setText("%.0f" % self.vehicle.actual_speed)
        self.redSpeedText1.setPos(posr + 10, 150)
        #self.blueSpeedText1.setText("%.0f" % self.vehicle.reference_speed)
        #self.blueSpeedText1.setPos(posb + 10, 130)
        a1 = np.array([posr, -100.], dtype = np.float)
        a2 = np.array([140.0, 140.], dtype = np.float)
        self.redSpeedBar.setData(a1, a2)
        self.redWheelHoriz.setData(r1, r2)
        self.redWheelVertic.setData(r0, r3)
        b1 = np.array([posb, -100.], dtype = np.float)
        b2 = np.array([120.0, 120.], dtype = np.float)
        #self.blueSpeedBar.setData(b1, b2)

        b1 = np.array([80 * np.cos(self.vehicle.wheel_reference_rad), -80 * np.cos(self.vehicle.wheel_reference_rad)], dtype = np.float).flatten()
        b2 = np.array([80 * np.sin(self.vehicle.wheel_reference_rad), -80 * np.sin(self.vehicle.wheel_reference_rad)], dtype = np.float).flatten()
        b0 = np.array([80 * np.cos(self.vehicle.wheel_reference_rad - np.math.pi / 2), 0.], dtype = np.float)
        b3 = np.array([80 * np.sin(self.vehicle.wheel_reference_rad - np.math.pi / 2), 0.], dtype = np.float)
        self.blueWheelHoriz.setData(b1, b2)
        self.blueWheelVertic.setData(b0, b3)


    def updateLabels(self):
        self.speedLabel.setText("actual: %4.1f km/h\nrefer : %4.1f km/h" % (self.vehicle.actual_speed, self.vehicle.reference_speed))        
        #self.isAutonomLabel.setText("x: %9.6f\ny: %9.6f" % (self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y))          

    def drawBlueCircle(self, to_plot):
        circle = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=8))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 80
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 80
        circle.setData(x, y)

    def drawRedCircle(self, to_plot):
        #circleB = pg.ScatterPlotItem(size = 8, pen = pg.mkPen(None), brush = pg.mkBrush(200, 66, 66))
        circle = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        to_plot.addItem(circle)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 100
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 100
        circle.setData(x, y)



class VehicleStatusSub(object):
    def __init__(self):
        rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", pacmod_m.SystemRptFloat, self.wheelDegCallBack)
        rospy.Subscriber("/pacmod/as_rx/accel_cmd", pacmod_m.SystemCmdFloat, self.vehicleAccelCmd)
        rospy.Subscriber("/pacmod/as_rx/brake_cmd", pacmod_m.SystemCmdFloat, self.vehicleBrakeCmd)
        rospy.Subscriber("/pacmod/as_tx/vehicle_speed", stdmsg.Float64, self.speedKmpHCallBack)
        rospy.Subscriber("/pacmod/as_rx/steer_cmd", pacmod_m.SteerSystemCmd, self.steerCmd)
        rospy.Subscriber("/pacmod/as_tx/enabled", stdmsg.Bool, self.vehicleStatusCallback)

        self.actual_speed = -1
        self.reference_speed = -1
        self.wheel_actual_rad = -0.01
        self.wheel_reference_rad =  0.01
        self.leaf_is_autonomous = "UNDEF"

    def wheelDegCallBack(self, msg_deg): 
        self.wheel_actual_rad = np.deg2rad(np.array([msg_deg.manual_input])) * 80 # wheel to steering ratio TODO

    def vehicleAccelCmd(self, msg):
        self.veh_accel_cmd = msg.command
        self.reference_speed = 10 * msg.command # TODO

    def vehicleBrakeCmd(self, msg):
        self.veh_brake_cmd = msg.command

    def speedKmpHCallBack(self, msg):
        self.actual_speed = np.array([msg.data]) * 3.6

    def steerCmd(self, msg):
        self.wheel_reference_rad = msg.command * 0.1

    def vehicleStatusCallback(self, msg):
        if msg.data == False: 
            self.leaf_is_autonomous = "IN CAR DRIVER"
        elif msg.data == True:
            self.leaf_is_autonomous = "YOU DRIVE"
        else:
            self.leaf_is_autonomous = "UNDEF"

if __name__ == "__main__":
    import sys
    rospy.init_node("plotter_", anonymous=True)
    rospy.loginfo("Vehicle status GUI started ... ")
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